import os
from openai import OpenAI

token = os.environ["GITHUB_TOKEN"]
endpoint = "https://models.inference.ai.azure.com"
model_name = "gpt-4o"

client = OpenAI(
    base_url=endpoint,
    api_key=token,
)

response = client.chat.completions.create(
    messages=[
        {
            "role": "system",
            "content": "You are a helpful assistant.",
        },
        {
            "role": "user",
            "content": "What is the capital of France?",
        }
    ],
    temperature=1.0,
    top_p=1.0,
    max_tokens=1000,
    model=model_name
)

print(response.choices[0].message.content)
## token: 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
## gpt_test2503: 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