from transformers import BitsAndBytesConfig 

GEMMA_3_1B = "google/gemma-3-1b-it"
QWEN_3_5_2B = "Qwen/Qwen3.5-2B"

QCONFIG_4B = BitsAndBytesConfig(load_in_4bit=True)
QCONFIG_8B = BitsAndBytesConfig(load_in_8bit=True)
