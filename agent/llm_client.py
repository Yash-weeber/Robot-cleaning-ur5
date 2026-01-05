import os
import time
import random
import threading
import ollama
from google import genai
from jinja2 import Environment, FileSystemLoader

_call_gemini_lock = threading.Lock()


class LLMInterface:
    def __init__(self, config):
        self.config = config
        # Load template from the agent/templates/ directory
        self.jinja_env = Environment(loader=FileSystemLoader(os.path.join('agent', 'templates')))
        self.template = self.jinja_env.get_template('llm_policy.j2')

        if not hasattr(self, "_active_idx"):
            self._active_idx = 0

    def render_prompt(self, iter_idx, feedback_text, bounds):
        """Renders the Jinja2 template with current data."""
        return self.template.render(
            max_iters=self.config['simulation']['max_iters'],
            n_bfs=self.config['dmp_params']['n_bfs'],
            total_weights=2 * self.config['dmp_params']['n_bfs'],
            xmin=bounds["xmin"],
            xmax=bounds["xmax"],
            ymin=bounds["ymin"],
            ymax=bounds["ymax"],
            optimum=0.0,
            step_size=self.config['dmp_params'].get('step_size', 50),
            feedback_text=feedback_text,
            iter_idx=iter_idx
        )

    def call_ollama(self, prompt):
        """Standard Ollama call as defined in original script."""
        try:
            response = ollama.chat(
                model=self.config['llm_settings']['ollama_model'],
                messages=[{"role": "user", "content": prompt}]
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

            print(f"Using {api_var} (index {api_index + 1}/{n})")
            try:
                client = genai.Client(api_key=api_key)
                for attempt in range(max_retries_per_key):
                    try:
                        resp = client.models.generate_content(
                            model=self.config['llm_settings']['gemini_model'],
                            contents=prompt
                        )
                        text = getattr(resp, "text", None) or str(resp)
                        with _call_gemini_lock:
                            self._active_idx = api_index
                        return text.strip()
                    except Exception as e:
                        last_error = e
                        # Decide if transient (429, 503, etc.)
                        s = str(e).lower()
                        if any(x in s for x in ["429", "503", "502", "504", "timeout", "unavailable"]):
                            sleep_time = min(base_wait_time * (backoff_factor ** attempt) + random.uniform(0, 1.5),
                                             max_sleep_cap)
                            print(f"Transient error: {e}. Retrying in {sleep_time:.1f}s...")
                            time.sleep(sleep_time)
                            continue
                        break
            except Exception as e:
                last_error = e
                continue

        raise RuntimeError(f"All Gemini API keys failed. Last error: {last_error}")