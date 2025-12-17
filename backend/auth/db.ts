import Database from "better-sqlite3";
import { drizzle } from "drizzle-orm/better-sqlite3";

const sqlite = new Database("auth.db");

sqlite.exec(`
CREATE TABLE IF NOT EXISTS user (
  id TEXT PRIMARY KEY,
  name TEXT NOT NULL,
  email TEXT UNIQUE NOT NULL,
  hashed_password TEXT,
  email_verified INTEGER DEFAULT 0,
  role TEXT,
  languages TEXT,
  language_skills TEXT
);

CREATE TABLE IF NOT EXISTS session (
  id TEXT PRIMARY KEY,
  user_id TEXT REFERENCES user(id),
  expires_at INTEGER,
  token TEXT UNIQUE
);
`);

export const db = drizzle(sqlite);
