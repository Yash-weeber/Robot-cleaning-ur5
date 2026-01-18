import os
import inspect
from typing import Optional, List, Union, Dict, Any

DEFAULT_MODEL = os.environ.get("OSS_120B_MODEL", "/scratch/melmisti/hf/models/gpt-oss-120b")


def call_oss_120b_mxfp4_with_vllm(
    prompt: str,
    *,
    model: str = DEFAULT_MODEL,
    tensor_parallel_size: int = 2,
    max_new_tokens: int = 256,
    temperature: float = 0.8,
    top_p: float = 0.95,
    seed: Optional[int] = None,
    stop: Optional[Union[str, List[str]]] = None,
    trust_remote_code: bool = True,
    dtype: str = "bfloat16",
    gpu_memory_utilization: float = 0.90,
    extra_llm_kwargs: Optional[Dict[str, Any]] = None,
) -> str:
    """
    Call an MXFP4-quantized OSS 120B model using vLLM (in-process).

    Important:
      - This requires vLLM to support MXFP4. Many builds do NOT.
      - We detect whether vLLM exposes `quantization=` and fail fast if not.
    """
    try:
        from vllm import LLM, SamplingParams  # type: ignore
    except Exception as e:
        raise RuntimeError(
            "vLLM is not installed/importable in this environment. "
            "Install vLLM (matching your CUDA/driver) and try again."
        ) from e

    llm_init_params = inspect.signature(LLM.__init__).parameters
    supports_quantization_arg = "quantization" in llm_init_params

    if not supports_quantization_arg:
        raise RuntimeError(
            "Your installed vLLM does not expose `LLM(..., quantization=...)`, so MXFP4 "
            "cannot be enabled from this API. "
            "Check your vLLM version/build or use a supported quantization method."
        )

    llm_kwargs: Dict[str, Any] = dict(
        model=model,
        trust_remote_code=trust_remote_code,
        dtype=dtype,
        tensor_parallel_size=int(tensor_parallel_size),
        gpu_memory_utilization=float(gpu_memory_utilization),
        quantization="mxfp4",
    )
    if extra_llm_kwargs:
        llm_kwargs.update(extra_llm_kwargs)

    llm = LLM(**llm_kwargs)

    sampling = SamplingParams(
        max_tokens=int(max_new_tokens),
        temperature=float(temperature),
        top_p=float(top_p),
        seed=seed,
        stop=stop,
    )

    outputs = llm.generate([prompt], sampling)
    return outputs[0].outputs[0].text


if __name__ == "__main__":
    tp = int(os.environ.get("VLLM_TP", "2"))
    print(
        call_oss_120b_mxfp4_with_vllm(
            "Explain quantum mechanics concisely.\n",
            tensor_parallel_size=tp,
            max_new_tokens=200,
            temperature=0.7,
            top_p=0.95,
        )
    )