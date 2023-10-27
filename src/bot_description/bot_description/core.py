from typing import List, Callable

from pydantic import Field, BaseModel
from promptulate.llms import ChatOpenAI, BaseLLM
from promptulate.agents import BaseAgent
from promptulate.output_formatter import OutputFormatter
from promptulate.schema import MessageSet, SystemMessage, UserMessage, BaseMessage
from promptulate.utils.color_print import print_text

from bot_description.prompt import (
    SYSTEM_PROMPT_TEMPLATE,
    GENERATE_PLAN_SYSTEM_PROMPT,
)
from bot_description.schema import (
    Task,
    GeneratePlanResponse,
    CommandResponse,
    Operator,
    Sensor,
)


class RobotController:
    def __init__(self, operators: List[Operator]) -> None:
        self.operators: List[Operator] = operators

    def execute_action(self, action_name: str, *args, **kwargs) -> None:
        operator = self.get_operator(action_name)
        print_text(f"[command] run {action_name} {str(args)} {str(kwargs)}", "green")
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
    def __init__(
        self, controller: RobotController, observer: RobotObserver, *args, **kwargs
    ):
        super().__init__(*args, **kwargs)
        self.llm = ChatOpenAI(temperature=0.0)
        self.robot_controller: RobotController = controller
        self.robot_observer: RobotObserver = observer
        self.user_demand: str = ""
        self.pending_tasks: List[Task] = []
        self.completed_tasks: List[Task] = []

    def get_llm(self) -> BaseLLM:
        return self.llm

    def _run(self, prompt: str, *args, **kwargs) -> str:
        """Main loop for RobotAgent"""
        self.user_demand = prompt

        # generate plan
        self.generate_tasks()

        # build system prompt
        system_prompt: str = SYSTEM_PROMPT_TEMPLATE.format(
            user_input=self.user_demand,
            task_queue=f"{str(self.pending_tasks)}",
        )
        formatter = OutputFormatter(CommandResponse)
        instruction = formatter.get_formatted_instructions()
        system_prompt = f"{system_prompt}\n{instruction}"

        messages: MessageSet = MessageSet(
            messages=[
                SystemMessage(content=system_prompt),
            ]
        )

        while True:
            # get current task
            # TODO add retry if failed
            llm_output: BaseMessage = self.llm.predict(messages)
            messages.add_message(llm_output)

            try:
                task: Task = formatter.formatting_result(llm_output.content).task

                print_text(f"[task] current task: {task.dict()}", "blue")
                if task.name == "stop":
                    self.robot_controller.execute_action("stop")
                    return

                # run task
                self.robot_controller.execute_action(
                    action_name=task.name, **task.parameters
                )
            except Exception as e:
                self.robot_controller.execute_action("stop")
                return

    def generate_tasks(self):
        """Generate tasks based on user demand."""
        # build conversation
        formatter = OutputFormatter(GeneratePlanResponse)
        instruction: str = formatter.get_formatted_instructions()
        messages: MessageSet = MessageSet(
            messages=[
                SystemMessage(content=f"{GENERATE_PLAN_SYSTEM_PROMPT}\n{instruction}"),
                UserMessage(content=f"##User input:\n{self.user_demand}"),
            ]
        )

        llm_output: str = self.llm.predict(messages).content

        response: GeneratePlanResponse = formatter.formatting_result(llm_output)
        self.pending_tasks = response.tasks
        print_text(f"Generate plans: {self.pending_tasks}", "green")


def main():
    operators = []
    sensors = []
    controller = RobotController(operators)
    observer = RobotObserver(sensors)
    robot_agent = RobotAgent(controller=controller, observer=observer)
