import rclpy 
from agent_controller.agent.agent import LLMAgent
from agent_controller.agent.helpers import (
    get_llm_message
)
from agent_controller.agent.prompts import (
    SYSTEM_PROMPT, 
    get_prompt
)

from agent_controller.tests.state_tests import (
    STATE_NOMINAL,
    STATE_FAULTY,
    STATE_DROPOUT
)


"""def main(args=None):
    rclpy.init(args=args)
    node = MainAgentNode()

    rclpy.spin(node=node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()"""


def main():
    agent = LLMAgent()
    prompt = get_prompt(state = STATE_DROPOUT)

    messages = get_llm_message(system=SYSTEM_PROMPT, prompt=prompt)

    print(agent.response(messages=messages))

if __name__=="__main__":
    main()