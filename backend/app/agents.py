import google.generativeai as genai
from tenacity import retry, stop_after_attempt, wait_exponential
from . import config

# Note: The Gemini client is configured in services.py, so we don't need to configure it again here.
# We will create a new model instance for chat.
try:
    gemini_chat_model = genai.GenerativeModel(config.GEMINI_CHAT_MODEL)
except Exception as e:
    print(f"Could not initialize Gemini Model in agents.py: {e}")
    gemini_chat_model = None

# Agent Skills
class AgentSkills:
    @staticmethod
    @retry(stop=stop_after_attempt(3), wait=wait_exponential(multiplier=1, min=4, max=10))
    def summarize_text(text: str, length: str = "medium") -> str:
        """Summarizes the given text using Gemini."""
        if not gemini_chat_model:
            return "Error: Gemini chat model is not available."
        try:
            # Gemini doesn't have a dedicated summarize endpoint, so we instruct the chat model.
            # The 'length' parameter can be translated into the prompt.
            prompt = f"Summarize the following text in a {length} paragraph:\n\n{text}"
            response = gemini_chat_model.generate_content(prompt)
            return response.text
        except Exception as e:
            # Re-raise to trigger tenacity's retry
            raise

    @staticmethod
    @retry(stop=stop_after_attempt(3), wait=wait_exponential(multiplier=1, min=4, max=10))
    def get_definition(term: str, context: str) -> str:
        """Gets a definition of a term from the context using Gemini."""
        if not gemini_chat_model:
            return "Error: Gemini chat model is not available."
        try:
            # The preamble from the old code is now part of the main prompt.
            prompt = f"You are a helpful assistant that provides definitions based on context. Based on the following text, define the term '{term}'.\n\nContext:\n{context}"
            response = gemini_chat_model.generate_content(prompt)
            return response.text
        except Exception as e:
            # Re-raise for tenacity
            raise

# Subagents
class SummaryAgent:
    def run(self, message: str, context: str) -> str:
        """Runs the summary agent."""
        if not context:
            return "Please select some text to summarize."
        return AgentSkills.summarize_text(context)

class DefinitionAgent:
    def run(self, message: str, context: str) -> str:
        """Runs the definition agent."""
        parts = message.lower().split("define")
        if len(parts) < 2:
            parts = message.lower().split("what is")
            if len(parts) < 2:
                return "I couldn't identify the term to define. Please use 'define [term]' or 'what is [term]'."

        term = parts[1].strip().replace("?","")
        
        if not context:
            return f"Please select some text to get a definition of '{term}'."
        return AgentSkills.get_definition(term, context)

# Router
subagent_routing_map = {
    "summarize": SummaryAgent,
    "summary": SummaryAgent,
    "define": DefinitionAgent,
    "what is": DefinitionAgent,
}

def route_to_subagent(message: str):
    """Routes a message to the appropriate subagent."""
    for keyword, agent_class in subagent_routing_map.items():
        if keyword in message.lower():
            return agent_class()
    return None
