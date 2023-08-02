from threading import Thread, Event
from typing import Any, Callable, Iterable, Mapping, List

class SynchronizerThread(Thread):
    """Thread class with Exception propagation and on event startup utils 
    Inherits from threading.Thread
    """
    def __init__(self, event: Event = None, group: None = None, target: Callable[..., object] = None, name: str = None, args: Iterable[Any] = ..., kwargs: Mapping[str, Any] = None, *, daemon: bool= None) -> None:
        super().__init__(group, target, name, args, kwargs, daemon=daemon)
        self.event = event

    def run(self):
        self.exc = None
        if self.event != None:
            self.event.wait()
        try:
            if hasattr(self, '_Thread__target'):
                # Thread uses name mangling prior to Python 3.
                self.ret = self._Thread__target(*self._Thread__args, **self._Thread__kwargs)
            else:
                self.ret = self._target(*self._args, **self._kwargs)
        except BaseException as e:
            self.exc = e

    def join(self, timeout=None):
        super(SynchronizerThread, self).join(timeout)
        if self.exc:
            raise self.exc
        return self.ret


def launch_simoultaneously(tasks: List[dict], error_message: str, use_event: bool = False):
    """Launch 

    Args:
        tasks (List[dict]): [{"name", "target", "args", "kwargs"}]
        error_message (str): Message to be displayed on error, thread name and Exception will be added at the ent
        use_event (bool, optional): when true will use an event to synchronize threads start. Defaults to False.
    """
    event = None
    if use_event:
        event = Event()

    threads = [
            SynchronizerThread(
                name=entry["name"],
                target=entry["target"],
                args=entry["args"],
                kwargs=entry["kwargs"],
                event=event
            )
            for entry in tasks
        ]
        
    [thread.start() for thread in threads]
    
    if use_event:
        event.set()
    
    for thread in threads:
        try:
            thread.join()
        except Exception as e:
            print(error_message + "{} : {} ".format(thread.name, e))