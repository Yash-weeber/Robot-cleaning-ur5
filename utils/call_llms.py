import ollama
import dotenv
from google import genai
import os
import threading
import random
import time
import torch
from transformers import AutoTokenizer, AutoModelForCausalLM, BitsAndBytesConfig

_call_gemini_lock = threading.Lock()

keys_file_path = "./keys.env"
API_KEYS = dotenv.dotenv_values(keys_file_path)
dotenv.load_dotenv(keys_file_path)
GEMINI_MODEL = "gemini-2.5-flash"  # use Pro or gemini-2.0-flash if you want faster/cheaper
GEMINI = genai.Client(api_key=os.environ.get("GOOGLE_API_KEY_1"))
# print("Loaded API keys:", API_KEYS)

os.environ["MUJOCO_GL"] = "egl"
OLLAMA_MODEL = "gpt-oss:120b"

def call_ollama(prompt: str, model:str = OLLAMA_MODEL, token_limit=64000) -> str:
    """
    Call Ollama LLM with the given prompt and return the text response.
    """
    try:
        # pdb.set_trace()
        # breakpoint()
        response = ollama.chat(
            model=model,
            messages=[{"role": "user", "content": prompt}],
            options={'num_ctx': token_limit, 'num_predict': 8192, 'temperature': 0.4},#, "num_gpu": 37},
            )
        # Ollama returns a dict with a 'message' key containing another dict with 'content'
        # breakpoint()
        return response["message"]["content"].strip()
    except Exception as e:
        raise RuntimeError(f"Ollama API call failed: {e}")
    
def call_gemini(prompt: str) -> str:
    """
    Gemini call with exponential backoff and persistent round-robin key rotation.
    - Starts from the last successful key across calls.
    - On success: pointer stays on that key.
    - If all keys fail in this call: pointer advances by 1 (keeps the cycle moving).
    - Adds thread safety, broader transient error detection, and capped backoff.
    """
    API_KEYS = [
        "GOOGLE_API_KEY_1",
        "GOOGLE_API_KEY_2",
        "GOOGLE_API_KEY_6",
        "GOOGLE_API_KEY_3",
        "GOOGLE_API_KEY_4",
        "GOOGLE_API_KEY_5",
    ]

    # Tuning: keep retries modest to avoid long stalls on a single key
    max_retries_per_key = 7
    base_wait_time = 4
    backoff_factor = 2
    max_sleep_cap = 45  # cap each sleep to avoid runaway waits

    n = len(API_KEYS)
    if n == 0:
        raise RuntimeError("No API key variables configured.")

    # Initialize persistent pointer if missing
    if not hasattr(call_gemini, "_active_idx"):
        call_gemini._active_idx = 0

    with _call_gemini_lock:
        start_idx = call_gemini._active_idx % n

    # Helper: decide if transient
    def _retryable(err: Exception) -> bool:
        s = str(err).lower()
        # Include rate limits, common 5xx/service errors, and timeouts/resets
        return (
            "429" in s or
            "503" in s or
            "502" in s or
            "504" in s or
            "temporarily unavailable" in s or
            "timeout" in s or
            "timed out" in s or
            "connection reset" in s or
            "econnreset" in s or
            "unavailable" in s
        )

    last_error = None

    for offset in range(n):
        api_index = (start_idx + offset) % n
        api_var = API_KEYS[api_index]
        api_key = os.environ.get(api_var)

        if not api_key:
            print(f"⚠️ {api_var} not found in environment, skipping...")
            continue

        print(f"🔑 Using {api_var} (index {api_index + 1}/{n})")
        try:
            client = genai.Client(api_key=api_key)
        except Exception as e:
            print(f"❌ Failed to initialize client for {api_var}: {e}")
            last_error = e
            continue

        for attempt in range(max_retries_per_key):
            try:
                resp = client.models.generate_content(
                    model=GEMINI_MODEL,
                    contents=prompt,
                    # config={"temperature": 0.2},
                )
                text = getattr(resp, "text", None) or str(resp)

                # Record success pointer
                with _call_gemini_lock:
                    call_gemini._active_idx = api_index
                return text.strip()

            except Exception as e:
                last_error = e
                if _retryable(e):
                    sleep_time = min(base_wait_time * (backoff_factor ** attempt) + random.uniform(0, 1.5),
                                     max_sleep_cap)
                    print(
                        f"⚠️ Transient Gemini error on {api_var}: {e}. "
                        f"Retrying in {sleep_time:.1f}s... ({attempt + 1}/{max_retries_per_key})"
                    )
                    time.sleep(sleep_time)
                    continue
                else:
                    print(f"❌ Non-retryable error on {api_var}: {e}")
                    break

        print(f"🔁 {api_var} exhausted after {max_retries_per_key} retries. Trying next key...")

    # All keys failed in this call; advance the pointer so the next call starts at the next key
    with _call_gemini_lock:
        call_gemini._active_idx = (start_idx + 1) % n

    raise RuntimeError(f"All Gemini API keys failed after rotation. Last error: {last_error}")
