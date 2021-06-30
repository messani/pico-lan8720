#ifndef MSGLOOP_H
#define MSGLOOP_H

#include <pico/sync.h>

typedef void (*eventloop_callback_t)(void *userdata);

struct eventloophandler
{
    eventloop_callback_t callback;
    void *userdata;
    struct eventloophandler * volatile next;
};

struct eventloop
{
  struct eventloophandler * volatile first;
  struct eventloophandler * volatile last;
  struct critical_section critical_section;
  bool exit;
};

void eventloop_init(struct eventloop *eventloop);
void eventloop_run(struct eventloop *eventloop);
void eventloop_enqueue(struct eventloop *eventloop, struct eventloophandler *handler);
void eventloophandler_setup(struct eventloophandler *eventloophandler, eventloop_callback_t callback, void *userdata);

#endif // MSGLOOP_H