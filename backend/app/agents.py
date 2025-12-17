from typing import Optional

# ---- Agent Skills ----
class AgentSkills:
    @staticmethod
    def summarize_text(text: str, length: str = "medium") -> str:
        return f"Summary ({length}): {text[:200]}..."

    @staticmethod
    def get_definition(term: str, context: str) -> str:
        if not context:
            return f"I don't know; the selected text does not contain the answer."
        return f"Definition of '{term}' based on context: {context[:200]}..."

# ---- Subagents ----
class SummaryAgent:
    def run(self, message: str, context: Optional[str] = None) -> str:
        if not context:
            return "Please select some text to summarize."
        return AgentSkills.summarize_text(context)

class DefinitionAgent:
    def run(self, message: str, context: Optional[str] = None) -> str:
        parts = message.lower().split("define")
        if len(parts) < 2:
            parts = message.lower().split("what is")
            if len(parts) < 2:
                return "I couldn't identify the term to define. Use 'define [term]' or 'what is [term]'."
        term = parts[1].strip().replace("?", "")
        if not context:
            return f"Please select text from the book to define '{term}'."
        return AgentSkills.get_definition(term, context)

# ---- Router ----
subagent_routing_map = {
    "summarize": SummaryAgent,
    "summary": SummaryAgent,
    "define": DefinitionAgent,
    "what is": DefinitionAgent,
}

def route_to_subagent(message: str):
    for keyword, agent_class in subagent_routing_map.items():
        if keyword in message.lower():
            return agent_class()
    return None
