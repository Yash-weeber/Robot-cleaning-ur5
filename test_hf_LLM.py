import os
import time
import torch
import importlib
from transformers import AutoConfig, AutoTokenizer, AutoModelForCausalLM

os.environ.setdefault("TRITON_CACHE_DIR", "/scratch/melmisti/triton_cache")
os.environ.setdefault("TORCH_EXTENSIONS_DIR", "/scratch/melmisti/torch_extensions")

# Check MXFP4 quantizer module
spec = importlib.util.find_spec('transformers.quantizers.quantizer_mxfp4')
print('MXFP4 quantizer module exists:', spec is not None)

# Check Triton explicitly
try:
    import triton
    print('Triton version:', triton.__version__)
    print('Triton >= 3.4.0:', tuple(map(int, triton.__version__.split('.'))) >= (3, 4, 0))
except ImportError as e:
    print('❌ Triton import failed:', e)

# Check what Transformers' MXFP4 quantizer sees
try:
    import transformers.quantizers.quantizer_mxfp4 as mxfp4_mod
    print("quantizer_mxfp4 file:", mxfp4_mod.__file__)
    candidates = ["Mxfp4Config", "MXFP4Config", "Mxfp4QuantizationConfig", "MXFP4QuantizationConfig"]
    found = [name for name in candidates if hasattr(mxfp4_mod, name)]
    print("MXFP4-related symbols found:", found)
    if found:
        Mxfp4Config = getattr(mxfp4_mod, found[0])
        print(f"Using {found[0]} from quantizer_mxfp4")
    else:
        print("❌ No known MXFP4 config symbol found in this Transformers build.")
except Exception as e:
    print("❌ MXFP4 module check failed:", e)

MODEL_DIR = "/scratch/melmisti/hf/models/gpt-oss-120b"

def main():
    print("\ntorch:", torch.__version__, "cuda:", torch.version.cuda)
    print("cuda available:", torch.cuda.is_available(), "gpus:", torch.cuda.device_count())

    cfg = AutoConfig.from_pretrained(MODEL_DIR, local_files_only=True, trust_remote_code=True)
    qcfg = getattr(cfg, "quantization_config", None)
    print("quantization_config:", qcfg)

    tok = AutoTokenizer.from_pretrained(MODEL_DIR, local_files_only=True, trust_remote_code=True)

    t0 = time.time()
    print("\n⚠️  Watch for 'dequantizing to bf16' warning below:")
    model = AutoModelForCausalLM.from_pretrained(
        MODEL_DIR,
        local_files_only=True,
        device_map="auto",
        dtype=torch.bfloat16,  # use 'dtype' not 'torch_dtype' to avoid deprecation
        trust_remote_code=True,
    )
    model.eval()
    print(f"✓ Model loaded in {time.time() - t0:.1f}s")

    messages = [{"role": "user", "content": "Explain quantum mechanics concisely."}]
    prompt = tok.apply_chat_template(messages, tokenize=False, add_generation_prompt=True)

    if torch.cuda.is_available():
        torch.cuda.reset_peak_memory_stats()

    inputs = tok(prompt, return_tensors="pt")
    inputs = {k: v.to(model.device) for k, v in inputs.items()}

    print("\nGenerating...")
    t1 = time.time()
    with torch.no_grad():
        out = model.generate(**inputs, max_new_tokens=200, do_sample=True, temperature=1.0)
    dt = time.time() - t1

    text = tok.decode(out[0], skip_special_tokens=True)
    print("\n=== Generated ===")
    print(text)
    print(f"\nGenerate time: {dt:.2f}s")

    if torch.cuda.is_available():
        peak_gb = torch.cuda.max_memory_allocated() / 1e9
        print(f"Peak CUDA memory: {peak_gb:.2f} GB")
        
        if peak_gb > 200:
            print("❌ MXFP4 NOT working (dequantized to bf16 - memory too high)")
        elif peak_gb < 120:
            print("✅ MXFP4 IS WORKING (memory consistent with quantization)")
        else:
            print("⚠️  Unclear - check warnings above")

if __name__ == "__main__":
    main()