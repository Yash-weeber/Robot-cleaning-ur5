import os

os.environ["VLLM_USE_V1"] = "0"

from vllm import LLM, SamplingParams

model_path = "/scratch/yrakesh1/modelhf/gpt-oss-120b/original"


llm = LLM(
    model=model_path,
    tensor_parallel_size=2,
    quantization="bitsandbytes", 
    gpu_memory_utilization=0.85,
    trust_remote_code=True,
    max_model_len=512,

    load_format="safetensors", 
)

sampling_params = SamplingParams(temperature=0.7, max_tokens=100)
prompt = "Reasoning: high\nUser: Propose 5 DMP weights "
outputs = llm.generate([prompt], sampling_params)

print(f"\n--- Output ---\n{outputs[0].outputs[0].text}")