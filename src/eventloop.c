#include "eventloop.h"
#include <stddef.h>
#include <stdio.h>

static struct eventloophandler *eventloop_dequeue(struct eventloop *eventloop)
{
  critical_section_enter_blocking(&eventloop->critical_section);
  struct eventloophandler *first = eventloop->first;
  struct eventloophandler *next = first->next;
  eventloop->first = next;
  if (eventloop->last == first)
    eventloop->last = next;
  critical_section_exit(&eventloop->critical_section);

  first->next = NULL;

  return first;
}

void eventloop_init(struct eventloop *eventloop)
{
  eventloop->first = NULL;
  eventloop->last = NULL;
  critical_section_init(&eventloop->critical_section);
  eventloop->exit = false;
}

static struct eventloophandler *lasthandler;

void eventloop_run(struct eventloop *eventloop)
{
  while (!eventloop->exit)   
  {
    if (eventloop->first == NULL)
      continue;

    struct eventloophandler *handler = eventloop_dequeue(eventloop);
    lasthandler = handler;
    handler->callback(handler->userdata);
  }
}

void eventloop_enqueue(struct eventloop *eventloop, struct eventloophandler *handler)
{
  critical_section_enter_blocking(&eventloop->critical_section);
  if(eventloop->first == NULL)
  {
    eventloop->first = handler;
  }
  else
  {
    eventloop->last->next = handler;
  }

  eventloop->last = handler;
  critical_section_exit(&eventloop->critical_section);
}

void eventloophandler_setup(struct eventloophandler *eventloophandler, eventloop_callback_t callback, void *userdata)
{
  eventloophandler->callback = callback;
  eventloophandler->userdata = userdata;
  eventloophandler->next = NULL;
} 