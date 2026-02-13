import asyncio
import json
from dotenv import load_dotenv
from paranet_agent import actor, connector
from paranet_agent.actor import BaseActor
from langchain_openai import AzureChatOpenAI
from langchain_core.messages import HumanMessage, SystemMessage
from langchain_core.prompts import PromptTemplate

@actor.type
class ChatGPTResponse:
    response: str

@actor.type
class TemplatePredictResponse:
    text: str

@actor.actor(name="chatgpt_py")
class ChatgptPy(BaseActor):
    """
    ChatGPT Python SDK Actor.
    
    Requires AZURE_OPENAI_API_KEY and AZURE_OPENAI_ENDPOINT env vars.
    """

    def __init__(self):
        """Initialize with Azure OpenAI client."""
        super().__init__()
        load_dotenv('../../.env')
        
        self.llm = AzureChatOpenAI(
            azure_deployment="gpt-4.1",
            api_version="2025-01-01-preview",
            temperature=0,
            max_tokens=None,
            timeout=None,
            max_retries=2
        )

    @actor.skill(subject="chatgpt_py", action="direct_chat")
    def direct_chat(self, prompt: str) -> ChatGPTResponse:
        """
        Direct LLM interaction without any system prompts or modifications.
        
        Args:
            prompt: User question/statement
        
        Returns:
            ChatGPTResponse with raw LLM response
        """
        message = HumanMessage(content=prompt)
        
        try:
            result = self.llm.invoke([message])
            return ChatGPTResponse(response=result.content)
        except Exception as e:
            return ChatGPTResponse(response=f"Error: {str(e)}")

    @actor.skill(subject="chatgpt_py", action="medical_chat")
    def medical_chat(self, prompt: str) -> ChatGPTResponse:
        """
        Direct LLM interaction with medical assistant system prompt.
        
        Args:
            prompt: User question/statement
            
        Returns:
            ChatGPTResponse with medical assistant response
        """
        messages = [
            SystemMessage(content="You are a medical assistant."),
            HumanMessage(content=prompt)
        ]
        
        try:
            result = self.llm.invoke(messages)
            return ChatGPTResponse(response=result.content)
        except Exception as e:
            return ChatGPTResponse(response=f"Error: {str(e)}")

    @actor.skill(subject="chatgpt_py")
    def template_predict(self, template: str, values: str) -> TemplatePredictResponse:
        """
        Uses LangChain PromptTemplate to format template with JSON values,
        then sends to LLM.
        
        Args:
            template: Prompt template string with placeholders
            values: JSON string with template values
            
        Returns:
            TemplatePredictResponse with generated text
        """
        print(f"TEMPLATE: {template}")
        print(f"VALUES: {values}")
        
        try:
            values_dict = json.loads(values)
            prompt = PromptTemplate.from_template(template)
            formatted_text = prompt.format(**values_dict)
            response = self.llm.invoke(formatted_text).content.strip()
            
            print(f"LLM RESPONSE({response})")
            return TemplatePredictResponse(text=response)
            
        except Exception as e:
            error_msg = f"Error in predict: {str(e)}"
            print(error_msg)
            return TemplatePredictResponse(text=error_msg)

# Actor registration and deployment
actor.register_actor(ChatgptPy())
connector.start()
actor.deploy('chatgpt_py', restart=True)
loop = asyncio.get_event_loop()
loop.run_until_complete(connector.get_task())