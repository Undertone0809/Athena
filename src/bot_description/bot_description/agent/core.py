from typing import List, Callable

from pydantic import Field, BaseModel
from promptulate.llms import ChatOpenAI, BaseLLM
from promptulate.agents import BaseAgent
from promptulate.output_formatter import OutputFormatter

from bot_description.agent.prompt import (
    SYSTEM_PROMPT_TEMPLATE,
    GENERATE_PLAN_SYSTEM_PROMPT_TEMPLATE,
)
from bot_description.agent.schema import Task, GeneratePlanResponse, CommandResponse


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


class RobotController:
    def __init__(self) -> None:
        self.operators: List[Operator] = []

    def execute_action(self, action_name: str, *args, **kwargs) -> None:
        operator = self.get_operator(action_name)
        operator.execute_action(*args, **kwargs)

    def get_operator(self, action_name: str) -> Operator:
        return next(
            (operator for operator in self.operators if operator.name == action_name),
            None,
        )


class RobotObserver:
    def __init__(self, sensors: List[Sensor]) -> None:
        self.sensors: List[Sensor] = sensors

    def get_data(self, sensor_name: str) -> None:
        sensor = self.get_sensor(sensor_name)
        return sensor.get_data()

    def get_sensor(self, sensor_name: str) -> Sensor:
        return next(
            (sensor for sensor in self.sensors if sensor.name == sensor_name),
            None,
        )


class RobotAgent(BaseAgent):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.llm = ChatOpenAI(temperature=0.0)
        self.robot_controller: RobotController
        self.robot_observer: RobotObserver
        self.user_demand: str = ""
        self.pending_tasks: List[Task] = []
        self.completed_tasks: List[Task] = []

    def _build_system_prompt() -> str:
        # TODO finish it
        formatter = OutputFormatter(CommandResponse)
        output_format: str = formatter.get_formatted_instructions()
        print(f"system prompt {output_format}")
        return SYSTEM_PROMPT_TEMPLATE.format(output_format=output_format)

    def get_llm(self) -> BaseLLM:
        return self.llm

    def _run(self, prompt: str, *args, **kwargs) -> str:
        """Main loop for RobotAgent"""
        self.user_demand = prompt

        # generate plan
        self.generate_tasks()

        while True:
            # generate conversation
            system_prompt: str = SYSTEM_PROMPT_TEMPLATE.format(
                user_input=self.user_demand,
                task_queue=f"{str(self.pending_tasks)}",
            )

            # get current task
            # TODO add retry if failed
            task: Task = self.llm(system_prompt).task

            if task.name == "stop":
                return

            # run task
            self.robot_controller.execute_action(
                action_name=task.name, **task.parameters
            )

    def generate_tasks(self):
        """Generate tasks based on user demand."""
        prompt: str = GENERATE_PLAN_SYSTEM_PROMPT_TEMPLATE.format(
            user_input=self.user_demand
        )
        llm_output: str = self.llm(prompt)

        formatter = OutputFormatter(GeneratePlanResponse)
        response: GeneratePlanResponse = formatter.formatting_result(llm_output)
        self.pending_tasks = response.tasks
        print(f"Generate plans: {self.pending_tasks}")


def main():
    operators = []
    sensors = []
    controller = RobotController(operators)
    observer = RobotObserver(sensors)
    robot_agent = RobotAgent(controller=controller, observer=observer)
