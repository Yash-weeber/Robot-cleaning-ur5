import os
import time
import random
import threading
import ollama
from google import genai
from jinja2 import Environment, FileSystemLoader

_call_gemini_lock = threading.Lock()

class LLMError(Exception):
    """Base exception for LLM client errors."""


class LLMInterface:
    def __init__(self, config):
        self.config = config

        template_dir = os.path.join('agent', 'templates')
        self.jinja_env = Environment(
            loader=FileSystemLoader(template_dir),
            autoescape=False,
            trim_blocks=True,
            lstrip_blocks=True,
        )

        if not hasattr(self, "_active_idx"):
            self._active_idx = 0

    def render_prompt(self, iter_idx, feedback_text, bounds, guidance_text="", scratchpad_text="", summarize=False):

        # Determine template name based on configuration flags
        traj_in_prompt = self.config['llm_settings'].get('traj_in_prompt', True)
        grid_reward = self.config['llm_settings'].get('grid_reward', False)
        if not summarize:
            template_name = self.config['llm_settings']['template']
        else:
            template_name = self.config['llm_settings']['summarizer_template']

        try:
            template = self.jinja_env.get_template(template_name)
        except Exception as e:
            raise RuntimeError(f"Failed to load Jinja2 template '{template_name}': {e}")

        # Rendering context synchronized with the Architect version
        return template.render(
            MAX_ITERS=self.config['simulation']['max_iters'],
            N_BFS=self.config['dmp_params']['n_bfs'],
            xmin=bounds["xmin"],
            xmax=bounds["xmax"],
            ymin=bounds["ymin"],
            ymax=bounds["ymax"],
            optimum=self.config['simulation'].get('optimum', 0),
            step_size=self.config['llm_settings'].get('step_size', 100),
            feedback_text=feedback_text,
            iter_idx=iter_idx,
            n_x_seg=self.config['dmp_params']['num_x_segments'],
            n_y_seg=self.config['dmp_params']['num_y_segments'],
            guidance_text=self.config['llm_settings'].get('guidance_file', ""),
            scratchpad_text=scratchpad_text
        )

    def call_ollama(self, prompt, token_limit=100000):

        try:
            response = ollama.chat(
                model=self.config['llm_settings']['llm_model'],
                messages=[{"role": "user", "content": prompt}],
                options={'num_ctx': token_limit}  # Mapping token_limit to Ollama's parameter
            )
            return response["message"]["content"].strip()
        except Exception as e:
            raise RuntimeError(f"Ollama API call failed: {e}")

    def call_gemini(self, prompt):

        api_keys = [
            "GOOGLE_API_KEY_1", "GOOGLE_API_KEY_2", "GOOGLE_API_KEY_6",
            "GOOGLE_API_KEY_3", "GOOGLE_API_KEY_4", "GOOGLE_API_KEY_5"
        ]

        max_retries_per_key = 7
        base_wait_time = 4
        backoff_factor = 2
        max_sleep_cap = 45

        n = len(api_keys)
        with _call_gemini_lock:
            start_idx = self._active_idx % n

        last_error = None
        for offset in range(n):
            api_index = (start_idx + offset) % n
            api_var = api_keys[api_index]
            api_key = os.environ.get(api_var)

            if not api_key:
                continue

            print(f" Using {api_var} (index {api_index + 1}/{n})")
            try:
                client = genai.Client(api_key=api_key)
                for attempt in range(max_retries_per_key):
                    try:
                        resp = client.models.generate_content(
                            model=self.config['llm_settings']['llm_model'],
                            contents=prompt
                        )
                        text = getattr(resp, "text", None) or str(resp)
                        with _call_gemini_lock:
                            self._active_idx = api_index
                        return text.strip()
                    except Exception as e:
                        last_error = e
                        s = str(e).lower()
                        if any(x in s for x in ["429", "503", "502", "504", "timeout", "unavailable"]):
                            sleep_time = min(base_wait_time * (backoff_factor ** attempt) + random.uniform(0, 1.5),
                                             max_sleep_cap)
                            print(f"Transient Gemini error: {e}. Retrying in {sleep_time:.1f}s...")
                            time.sleep(sleep_time)
                            continue
                        break
            except Exception as e:
                last_error = e
                continue
        raise RuntimeError(f"All Gemini API keys failed. Last error: {last_error}")
    
    def call_gemma(self, prompt: str, image_data: list[str] | str | None=None):
        try:
            import openai
        except ImportError as e:
            raise ImportError(
                "OpenAI SDK not installed. Run: pip install openai"
            ) from e
        api_key = os.environ.get("OPENAI_API_KEY")
        if not api_key:
            raise LLMError("OPENAI_API_KEY environment variable not set")
        self._openai = openai
        self._client = openai.OpenAI(
            api_key=api_key,
            base_url="https://openai.rc.asu.edu/v1"
        )

        import base64
        content = [{"type": "text", "text": prompt}]
        
        if image_data:
            # Handle list of base64 strings (New)
            if isinstance(image_data, list):
                for b64 in image_data:
                    content.append({
                        "type": "image_url",
                        "image_url": {"url": f"data:image/jpeg;base64,{b64}"}
                    })
            # Handle single file path (Legacy/Standard)
            elif isinstance(image_data, str):
                with open(image_data, "rb") as f:
                    b64 = base64.b64encode(f.read()).decode('utf-8')
                content.append({
                    "type": "image_url",
                    "image_url": {"url": f"data:image/png;base64,{b64}"}
                })

        response = self._client.chat.completions.create(
            model=self.config['llm_settings']['llm_model'],
            max_tokens=self.config['llm_settings'].get('max_tokens', 70000),
            temperature=self.config['llm_settings'].get('temperature', 0.7),
            messages=[{"role": "user", "content": content}],
        )

        if hasattr(response, 'usage') and response.usage:
            prompt_tokens = getattr(response.usage, 'prompt_tokens', 'Unknown')
            completion_tokens = getattr(response.usage, 'completion_tokens', 'Unknown')
            total_tokens = getattr(response.usage, 'total_tokens', 'Unknown')
            
            print("-" * 40)
            print("API Token Usage:")
            print(f"  -> Prompt (Images + Text): {prompt_tokens} tokens")
            print(f"  -> Completion (Response):  {completion_tokens} tokens")
            print(f"  -> Total Tokens Used:      {total_tokens} tokens")
            print("-" * 40)
            
        content = response.choices[0].message.content
        if content is None:
            raise LLMError("OpenAI API returned content=None, which is likely a transient issue. Consider retrying.")
        return content

