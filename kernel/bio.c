// Buffer cache.
//
// The buffer cache is a linked list of buf structures holding
// cached copies of disk block contents.  Caching disk blocks
// in memory reduces the number of disk reads and also provides
// a synchronization point for disk blocks used by multiple processes.
//
// Interface:
// * To get a buffer for a particular disk block, call bread.
// * After changing buffer data, call bwrite to write it to disk.
// * When done with the buffer, call brelse.
// * Do not use the buffer after calling brelse.
// * Only one process at a time can use a buffer,
//     so do not keep them longer than necessary.


#include "types.h"
#include "param.h"
#include "spinlock.h"
#include "sleeplock.h"
#include "riscv.h"
#include "defs.h"
#include "fs.h"
#include "buf.h"

#define NBUCKETS 13 // Prime number of hash buckets to reduce collisions

struct {
  struct buf buf[NBUF];  // The buffer array
  
  // Hash table buckets
  struct {
    struct spinlock lock;        // Per-bucket lock
    struct buf head;             // Dummy head for this bucket's list
  } buckets[NBUCKETS];
  
  struct spinlock eviction_lock; // Lock for buffer eviction
} bcache;

// Hash function to map block numbers to buckets
static uint
hash(uint dev, uint blockno)
{
  return (dev + blockno) % NBUCKETS;
}

void
binit(void)
{
  // char lockname[16];

  // Initialize eviction lock
  initlock(&bcache.eviction_lock, "bcache");
  
  // Initialize hash buckets
  for(int i = 0; i < NBUCKETS; i++){
    // snprintf(lockname, sizeof(lockname), "bcache.bucket%d", i);
    initlock(&bcache.buckets[i].lock, "bcache.bucket");
    
    bcache.buckets[i].head.prev = &bcache.buckets[i].head;
    bcache.buckets[i].head.next = &bcache.buckets[i].head;
  }

  // Initialize all buffers
  for(struct buf *b = bcache.buf; b < bcache.buf + NBUF; b++){
    b->next = b->prev = b;  // Not linked to any list yet
    initsleeplock(&b->lock, "buffer");
    b->refcnt = 0;
    b->valid = 0;
    
    // Add to bucket 0 initially (will be redistributed as needed)
    b->next = bcache.buckets[0].head.next;
    b->prev = &bcache.buckets[0].head;
    bcache.buckets[0].head.next->prev = b;
    bcache.buckets[0].head.next = b;
  }
}

// Look through buffer cache for block on device dev.
// If not found, allocate a buffer.
// In either case, return locked buffer.
static struct buf*
bget(uint dev, uint blockno)
{
  struct buf *b;
  uint bucket_id = hash(dev, blockno);
  struct buf *bucket_head = &bcache.buckets[bucket_id].head;

  acquire(&bcache.buckets[bucket_id].lock);
  
  // Is the block already cached?
  for(b = bucket_head->next; b != bucket_head; b = b->next){
    if(b->dev == dev && b->blockno == blockno){
      b->refcnt++;
      release(&bcache.buckets[bucket_id].lock);
      acquiresleep(&b->lock);
      return b;
    }
  }

  // Not cached. Look for an unused buffer in this bucket
  for(b = bucket_head->next; b != bucket_head; b = b->next){
    if(b->refcnt == 0) {
      b->dev = dev;
      b->blockno = blockno;
      b->valid = 0;
      b->refcnt = 1;
      release(&bcache.buckets[bucket_id].lock);
      acquiresleep(&b->lock);
      return b;
    }
  }
  release(&bcache.buckets[bucket_id].lock);
  
  // Need to steal a buffer from another bucket
  acquire(&bcache.eviction_lock);
  
  // Double-check our bucket in case something changed
  acquire(&bcache.buckets[bucket_id].lock);
  for(b = bucket_head->next; b != bucket_head; b = b->next){
    if(b->refcnt == 0) {
      b->dev = dev;
      b->blockno = blockno;
      b->valid = 0;
      b->refcnt = 1;
      release(&bcache.buckets[bucket_id].lock);
      release(&bcache.eviction_lock);
      acquiresleep(&b->lock);
      return b;
    }
  }
  release(&bcache.buckets[bucket_id].lock);
  
  // Try to find a buffer in another bucket
  struct buf *victim = 0;
  
  for(int i = 0; i < NBUCKETS; i++) {
    if(i == bucket_id) continue; // Skip our bucket
    
    struct buf *other_head = &bcache.buckets[i].head;
    acquire(&bcache.buckets[i].lock);
    
    for(b = other_head->next; b != other_head; b = b->next) {
      if(b->refcnt == 0) {
        victim = b;
        
        // Remove from current bucket's list
        b->next->prev = b->prev;
        b->prev->next = b->next;
        
        release(&bcache.buckets[i].lock);
        goto found_victim;
      }
    }
    
    release(&bcache.buckets[i].lock);
  }
  
  // No unused buffer found - shouldn't happen in xv6 design
  release(&bcache.eviction_lock);
  panic("bget: no buffers");

found_victim:
  // Add victim to our bucket
  acquire(&bcache.buckets[bucket_id].lock);
  victim->dev = dev;
  victim->blockno = blockno;
  victim->valid = 0;
  victim->refcnt = 1;
  
  // Add to our bucket's list
  victim->next = bucket_head->next;
  victim->prev = bucket_head;
  bucket_head->next->prev = victim;
  bucket_head->next = victim;
  
  release(&bcache.buckets[bucket_id].lock);
  release(&bcache.eviction_lock);
  
  acquiresleep(&victim->lock);
  return victim;
}

// Return a locked buf with the contents of the indicated block.
struct buf*
bread(uint dev, uint blockno)
{
  struct buf *b;

  b = bget(dev, blockno);
  if(!b->valid) {
    virtio_disk_rw(b, 0);
    b->valid = 1;
  }
  return b;
}

// Write b's contents to disk.  Must be locked.
void
bwrite(struct buf *b)
{
  if(!holdingsleep(&b->lock))
    panic("bwrite");
  virtio_disk_rw(b, 1);
}

// Release a locked buffer.
// Move to the head of the most-recently-used list.
void
brelse(struct buf *b)
{
  if(!holdingsleep(&b->lock))
    panic("brelse");

  releasesleep(&b->lock);

  uint bucket_id = hash(b->dev, b->blockno);
  
  acquire(&bcache.buckets[bucket_id].lock);
  b->refcnt--;
  release(&bcache.buckets[bucket_id].lock);
}

void
bpin(struct buf *b) {
  uint bucket_id = hash(b->dev, b->blockno);
  
  acquire(&bcache.buckets[bucket_id].lock);
  b->refcnt++;
  release(&bcache.buckets[bucket_id].lock);
}

void
bunpin(struct buf *b) {
  uint bucket_id = hash(b->dev, b->blockno);
  
  acquire(&bcache.buckets[bucket_id].lock);
  b->refcnt--;
  release(&bcache.buckets[bucket_id].lock);
}


