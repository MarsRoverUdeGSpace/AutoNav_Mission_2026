import asyncio 
from typing import Dict, Any 
from transformers import AutoTokenizer, BitsAndBytesConfig, AutoModelForCausalLM
import torch 
from agent_controller.settings import (
    QWEN_3_5_2B,
    GEMMA_3_1B, 
    QCONFIG_4B, 
    QCONFIG_8B)



class LLMAgent():
    """
    Initialize the agent with the LLM and all it's tool calling logic
    """

    def __init__(self):
        self.model = AutoModelForCausalLM.from_pretrained(
            QWEN_3_5_2B,
            quantization_config = QCONFIG_4B,
            device_map="cuda"
        ).eval()

        self.tokenizer = AutoTokenizer.from_pretrained(QWEN_3_5_2B)

    def response(self, messages):
        
        inputs = self.tokenizer.apply_chat_template(
            messages,
            add_generation_prompt=True,
            tokenize=True,
            return_dict=True,
            return_tensors="pt"
        ).to(self.model.device) 

        
        with torch.inference_mode():
            outputs = self.model.generate(
                **inputs,
                max_new_tokens=512, # Adjust this based on how long you want responses to be
                pad_token_id=self.tokenizer.eos_token_id # Prevents console warnings
            )

        
        input_length = inputs['input_ids'].shape[1]
        generated_tokens = outputs[0][input_length:]

        
        return self.tokenizer.decode(generated_tokens, skip_special_tokens=True)
        
         