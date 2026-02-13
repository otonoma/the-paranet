# LLM Integration Actor Demo

A demo showcasing three different approaches to LLM integration using Paraflow and the Paranet:

1. **Direct LLM Calls** - Raw conversational interface without system prompts
2. **Medical Assistant Chat** - Conversational interface with specialized medical system prompt
3. **Template-Based Classification** - Structured email classification and routing using LLM templates

## Deploy

**This project requires para 0.20 at a minimum. Check with `para -V`**

Before deploying, you need to set two environment variables to access Azure OpenAI. They can be written to a `.env` file in this folder:

```
AZURE_OPENAI_API_KEY=<your key here>
AZURE_OPENAI_ENDPOINT=<your endpoint here>
```

Login, deploy node and package:

```bash
para devkit login
para docker deploy node
para docker deploy package
```

## Architecture

The system consists of two main components:

1. **ChatGPT Python Actor** (`chatgpt_py`) - Handles LLM interactions via Azure OpenAI
   - `direct_chat` skill - Raw conversational interface without system prompts
   - `medical_chat` skill - Medical assistant with specialized system prompt
   - `template_predict` skill - Template-based structured prompting
2. **Paraflow Skills** - Orchestrates different interaction patterns
   - `chatgpt/direct_chat` - Direct conversation workflow
   - `chatgpt/medical_chat` - Medical assistant workflow
   - `email_classifier/prioritize` - Email classification and routing workflow

## Three Demo Examples

### Example 1: Direct LLM Conversation

**Skill**: `chatgpt/direct_chat`

Pure direct conversation with the LLM without any system prompts or modifications (raw LLM interaction).

#### Using Paracord:
1. Navigate to `Actor Hub` → `chatgpt` → `direct_chat` skill
2. Enter any question in the prompt field

**Result**: Raw conversational response from the LLM without any system context

### Example 2: Medical Assistant Chat

**Skill**: `chatgpt/medical_chat`

Conversational interface with a medical assistant system prompt for healthcare-related queries.

#### Using Paracord:
1. Navigate to `Actor Hub` → `chatgpt` → `medical_chat` skill
2. Enter any medical question in the prompt field

**Result**: Medical assistant response with professional healthcare context

### Example 3: Template-Based Email Classification

**Skill**: `email_classifier/prioritize` 

Demonstrates structured LLM prompting using templates and automated workflow routing.

#### Using Paracord:
1. Navigate to `Actor Hub` → `email_classifier` → `prioritize` skill  
2. Fill in email details:
   - **Subject**: Email subject line
   - **Body**: Email content
3. LLM will categorize the e-mail and paraflow will route e-mails based on its category. Actions for each category can be implemented here.


## Demo

https://github.com/user-attachments/assets/dffc6a2b-423d-4959-b38a-386f39887853

