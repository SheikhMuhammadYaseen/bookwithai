from fastapi import FastAPI
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware
from . import services, config

app = FastAPI()

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allows all origins
    allow_credentials=True,
    allow_methods=["*"],  # Allows all methods
    allow_headers=["*"],  # Allows all headers
)

class ChatRequest(BaseModel):
    message: str
    context: str | None = None

class ChatResponse(BaseModel):
    response: str

@app.post("/api/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    context = request.context
    if not context:
        # Fallback to vector search if no text is selected
        context = services.search_qdrant(request.message, config.QDRANT_COLLECTION_NAME)
        
    response_text = services.get_chat_response(request.message, context)
    
    return ChatResponse(response=response_text)

@app.get("/api/health")
async def health_check():
    return {"status": "ok"}

@app.get("/api/debug-config")
async def debug_config():
    return {
        "cohere_api_key_loaded": bool(config.COHERE_API_KEY),
        "qdrant_host_loaded": bool(config.QDRANT_HOST),
        "qdrant_api_key_loaded": bool(config.QDRANT_API_KEY),
        "database_url_loaded": bool(config.DATABASE_URL),
    }