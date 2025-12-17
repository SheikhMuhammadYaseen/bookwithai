// bookwithai/backend/auth/schema.ts
import { sqliteTable, text, integer } from "drizzle-orm/sqlite-core";

export const user = sqliteTable("user", {
  id: text("id").primaryKey(),
  name: text("name").notNull(),
  email: text("email").unique().notNull(),
  hashedPassword: text("hashed_password"),
  emailVerified: integer("email_verified", { mode: "boolean" }).default(false).notNull(),
  role: text("role"),
  languages: text("languages"),
  languageSkills: text("language_skills"),
  createdAt: integer("created_at", { mode: "timestamp_ms" }).notNull(),
  updatedAt: integer("updated_at", { mode: "timestamp_ms" }).notNull(),
});

export const session = sqliteTable("session", {
  id: text("id").primaryKey(),
  userId: text("user_id").references(() => user.id),
  expiresAt: integer("expires_at", { mode: "timestamp_ms" }),
  token: text("token").unique(),
});

export const schema = { user, session };
