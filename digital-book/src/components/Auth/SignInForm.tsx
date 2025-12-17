import React, { useState } from 'react';
import styles from './styles.module.css';

const AUTH_API_URL = 'http://localhost:3001/api/auth';
const ERROR_API_URL = 'http://localhost:8000';

const logError = async (error: Error) => {
  try {
    await fetch(`${ERROR_API_URL}/api/log-error`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ message: error.message, stack: error.stack }),
    });
  } catch {}
};

const SignInForm = () => {
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);

    try {
      const res = await fetch(`${AUTH_API_URL}/sign-in/email`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify({ email, password }),
      });

      if (!res.ok) {
        const text = await res.text();
        const error = new Error(text || 'Invalid email or password');
        await logError(error);
        alert(`Sign in failed: ${text || 'Server error'}`);
        setLoading(false);
        return;
      }

      alert('✅ Signed in successfully');
      window.location.href = '/';
    } catch (error: any) {
      await logError(error);
      alert('❌ Sign in Unsuccessful – Check server console');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className={styles.formContainer}>
      <form className={styles.form} onSubmit={handleSubmit}>
        <h2>Sign In</h2>
        <p className={styles.subtitle}>
          Welcome back to the Physical AI & Humanoid Robotics Learning Platform
        </p>

        <div className={styles.inputGroup}>
          <input
            type="email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
            disabled={loading}
            placeholder=" "
          />
          <label>Email Address</label>
        </div>

        <div className={styles.inputGroup}>
          <input
            type="password"
            value={password}
            onChange={(e) => setPassword(e.target.value)}
            required
            disabled={loading}
            placeholder=" "
          />
          <label>Password</label>
        </div>

        <button type="submit" disabled={loading}>
          {loading ? 'Signing in...' : 'Sign In'}
        </button>
      </form>
    </div>
  );
};

export default SignInForm;
