// Physical memory allocator, for user processes,
// kernel stacks, page-table pages,
// and pipe buffers. Allocates whole 4096-byte pages.

#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "riscv.h"
#include "defs.h"

void freerange(void *pa_start, void *pa_end);

extern char end[]; // first address after kernel.
                   // defined by kernel.ld.

struct run {
  struct run *next;
};

struct {
  struct spinlock lock;
  struct run *freelist;
} kmem[NCPU]; // Per-CPU memory allocator state

void
kinit()
{
  // char lockname[16];
  for(int i = 0; i < NCPU; i++) { // per-CPU locks
    // snprintf(lockname, sizeof(lockname), "kmem%d", i);
    initlock(&kmem[i].lock, "kmem");
  }
  freerange(end, (void*)PHYSTOP);
  // printf("kmem lock init: %s\n", kmem[0].lock.name);
}

void
freerange(void *pa_start, void *pa_end)
{
  char *p;
  p = (char*)PGROUNDUP((uint64)pa_start);
  for(; p + PGSIZE <= (char*)pa_end; p += PGSIZE)
    kfree(p);
}

// Free the page of physical memory pointed at by pa,
// which normally should have been returned by a
// call to kalloc().  (The exception is when
// initializing the allocator; see kinit above.)
void
kfree(void *pa)
{
  struct run *r;

  if(((uint64)pa % PGSIZE) != 0 || (char*)pa < end || (uint64)pa >= PHYSTOP)
    panic("kfree");

  // Fill with junk to catch dangling refs.
  memset(pa, 1, PGSIZE);

  r = (struct run*)pa;

  // Add the page to the current CPU's free list
  push_off();
  int cpu = cpuid();
  pop_off();

  acquire(&kmem[cpu].lock);
  r->next = kmem[cpu].freelist;
  kmem[cpu].freelist = r;
  release(&kmem[cpu].lock);
}

// Allocate one 4096-byte page of physical memory.
// Returns a pointer that the kernel can use.
// Returns 0 if the memory cannot be allocated.
void *
kalloc(void)
{
  struct run *r;

  // Disable interrupts to get the current CPU id
  push_off();
  int cpu = cpuid();
  pop_off();

  // Try to allocate from this CPU's free list
  acquire(&kmem[cpu].lock);
  r = kmem[cpu].freelist;
  if(r)
    kmem[cpu].freelist = r->next;
  release(&kmem[cpu].lock);

  // If this CPU's free list is empty, try to steal from other CPUs
  if(r == 0) {
    for(int i = 0; i < NCPU; i++) {
      if(i == cpu)
        continue;  // Skip the current CPU

      // Try to take half of another CPU's free list
      acquire(&kmem[i].lock);
      struct run *list = kmem[i].freelist;
      if(list) {
        // Count the number of pages in this free list
        int count = 0;
        struct run *current = list;
        while(current) {
          count++;
          current = current->next;
        }

        // If there's only one page, take it
        if(count == 1) {
          r = list;
          kmem[i].freelist = 0;
        } 
        // Otherwise steal half the pages
        else if(count > 1) {
          // Find the midpoint of the list
          struct run *mid = list;
          for(int j = 0; j < count/2 - 1; j++) {
            mid = mid->next;
          }

          // Take the second half
          r = mid->next;
          mid->next = 0;

          // Move all but one page to our CPU's free list
          if(r) {
            acquire(&kmem[cpu].lock);
            struct run *stolen = r->next;
            r->next = 0;  // Keep one page to return
            
            // Add the rest to our CPU's free list
            if(stolen) {
              struct run *last = stolen;
              while(last->next)
                last = last->next;
              last->next = kmem[cpu].freelist;
              kmem[cpu].freelist = stolen;
            }
            release(&kmem[cpu].lock);
          }
        }
      }
      release(&kmem[i].lock);
      
      if(r)
        break;  // found memory. no need to check other CPUs
    }
  }

  if(r)
    memset((char*)r, 5, PGSIZE); // fill with junk
  return (void*)r;
}
