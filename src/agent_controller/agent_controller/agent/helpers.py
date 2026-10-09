


def get_llm_message(system, prompt):
    messages = [
        {
            "role": "system",
            "content": system
        },
        {
            "role": "user",
            "content": prompt
        }
    ]

    return messages