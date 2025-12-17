<<<<<<< HEAD
from fastapi import FastAPI, Query, Depends
from pydantic import BaseModel
from sqlalchemy.orm import Session
from fastapi.middleware.cors import CORSMiddleware
from . import services, database

app = FastAPI(
    title="BookWithAI RAG Chatbot",
    description="A Retrieval-Augmented Generation chatbot for your book",
    version="1.0"
)

# ---- Database ----
database.Base.metadata.create_all(bind=database.engine)

# ---- CORS ----
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ---- Models ----
=======
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

>>>>>>> e25e880e08042cc52ec0e5c6e265e2e07042e8ac
class ChatRequest(BaseModel):
    message: str
    context: str | None = None

class ChatResponse(BaseModel):
    response: str

<<<<<<< HEAD
class ErrorLogCreate(BaseModel):
    message: str
    stack: str

# ---- Root ----
@app.get("/")
async def root():
    return {"message": "BookWithAI RAG Chatbot is running!"}

# ---- Health ----
=======
@app.post("/api/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    context = request.context
    if not context:
        # Fallback to vector search if no text is selected
        context = services.search_qdrant(request.message, config.QDRANT_COLLECTION_NAME)
        
    response_text = services.get_chat_response(request.message, context)
    
    return ChatResponse(response=response_text)

>>>>>>> e25e880e08042cc52ec0e5c6e265e2e07042e8ac
@app.get("/api/health")
async def health_check():
    return {"status": "ok"}

<<<<<<< HEAD
# ---- Chat ----
@app.post("/api/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    response_text = services.get_chat_response(
        message=request.message,
        context=request.context
    )
    return ChatResponse(response=response_text)

# ---- Error Logging ----
@app.post("/api/log-error")
async def log_error(error_log: ErrorLogCreate, db: Session = Depends(database.get_db)):
    db_error = database.ErrorLog(message=error_log.message, stack=error_log.stack)
    db.add(db_error)
    db.commit()
    db.refresh(db_error)
    return {"status": "error logged", "error_id": db_error.id}

# ---- Quick test ----
@app.get("/api/test")
async def test_chat(message: str = Query(...), context: str | None = None):
    response_text = services.get_chat_response(message, context)
    return {"message": message, "response": response_text}
=======
@app.get("/api/debug-config")
async def debug_config():
    return {
        "cohere_api_key_loaded": bool(config.COHERE_API_KEY),
        "qdrant_host_loaded": bool(config.QDRANT_HOST),
        "qdrant_api_key_loaded": bool(config.QDRANT_API_KEY),
        "database_url_loaded": bool(config.DATABASE_URL),
    }
>>>>>>> e25e880e08042cc52ec0e5c6e265e2e07042e8ac
