from typing import Dict, Any, List, Callable
from pydantic import BaseModel, Field


class Operator:
    def __init__(self, name: str, description: str, callback: Callable):
        self.name = name
        self.description = description
        self.callback = callback

    def execute_action(self, *args, **kwargs):
        self.callback(*args, **kwargs)


class Sensor:
    def __init__(self, name: str, description: str, callback: Callable):
        self.name = name
        self.description = description
        self.callback = callback

    def get_data(self, *args, **kwargs):
        self.callback(*args, **kwargs)


class Task(BaseModel):
    id: int = Field(description="Autoincrement task id")
    name: str = Field(description="task name")
    parameters: Dict[str, Any] = Field(description="task parameters")
    reason: str = Field(description="Reason for task execution")


class GeneratePlanResponse(BaseModel):
    tasks: List[Task] = Field(description="task sequences")


class CommandResponse(BaseModel):
    task: Task = Field(description="control task")


def get_task_examples():
    """Get task examples for output format few-shot."""
    return [
        Task(name="go_front", parameters=dict(distance=1)),
        Task(name="go_back", parameters=dict(distance=1.55)),
        Task(name="turn_left", parameters=dict(distance=30.0)),
    ]
