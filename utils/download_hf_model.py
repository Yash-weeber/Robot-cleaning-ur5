from huggingface_hub import snapshot_download

MODEL_ID = "openai/gpt-oss-120b"          # <- replace
LOCAL_DIR = "/scratch/melmisti/hf/models/gpt-oss-120b"  # <- your target folder

snapshot_download(
    repo_id=MODEL_ID,
    local_dir=LOCAL_DIR,
    local_dir_use_symlinks=False,  # make real files (portable)
)
print(f"Downloaded {MODEL_ID} to: {LOCAL_DIR}")