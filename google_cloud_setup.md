# Getting Started with Gemini on Vertex AI

Welcome to the project! This guide will help you set up your local development environment to start interacting with Gemini models.

## Prerequisites

1. **Accept Project Invitation:** Ensure you have accepted the email invitation to the Google Cloud project.
2. **Python Installed:** Ensure you have Python 3.10 or higher.

---

## 1. Install & Initialize Google Cloud CLI

The `gcloud` CLI is the primary tool for authenticating your machine with Google Cloud.

### Installation

- **macOS / Linux:**

  ```bash
  curl https://sdk.cloud.google.com | bash
  ```

  *(Restart your terminal after installation)*

- **Windows:** Download and run the [Google Cloud CLI Installer](https://cloud.google.com/sdk/docs/install#windows).

### Initialization

Connect the CLI to your account and the specific project:

```bash
gcloud init
```

Follow the browser prompts to log in and select the project ID you were invited to.

---

## 2. Set Up Authentication (ADC)

To run Python scripts locally without handling complex key files, use Application Default Credentials (ADC). This "links" your local Python environment to your `gcloud` login.

```bash
gcloud auth application-default login
```

A browser window will open. Log in and click "Allow."

---

## 3. Install the Python SDK

Install the official Google Cloud Vertex AI library in your project's virtual environment:

```bash
pip install google-cloud-aiplatform
```

---

## 4. Run Your First Script

Create a file named `hello_gemini.py` and replace `your-project-id` with the actual ID of the project you joined.

```python
from google import genai
from google.genai import types

client = genai.Client(vertexai=True, project="polar-ray-459012-b0", location="us-central1")
response = client.models.generate_content(
    model="gemini-2.5-flash",
    contents="Hello! What can you help me with?",
    )
print(response.text)
```

---

## Troubleshooting

- **`gcloud: command not found`:** Restart your terminal. If it still fails, ensure the installer added the Google Cloud SDK to your `PATH`.
- **`DefaultCredentialsError`:** This means the script can't find your login. Re-run `gcloud auth application-default login`.
- **Permission Denied:** Ensure your project administrator has assigned you the **Vertex AI User** role in the IAM console.