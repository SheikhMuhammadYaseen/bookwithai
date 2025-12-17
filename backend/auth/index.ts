import express from "express";
import cors from "cors";
import { toNodeHandler } from "better-auth/node";
import { auth } from "./auth.js";

const app = express();
const PORT = 3001;

app.use(cors({
  origin: ["http://localhost:8000", "http://localhost:3000"],
  credentials: true,
}));
app.use(express.json());

app.use("/api/auth", toNodeHandler(auth));

app.get("/", (req, res) => {
  res.json({ message: "Better Auth server running!" });
});

app.listen(PORT, () => {
  console.log(`Auth server running on http://localhost:${PORT}`);
});
