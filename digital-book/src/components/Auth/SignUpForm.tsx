import React, { useState } from 'react';
import styles from './styles.module.css';

const AUTH_API_URL = 'https://ys5555-au-aibook.hf.space/api/auth';

const roles = ['Student', 'Professional', 'Hobbyist', 'Researcher', 'Other'];
const programmingLanguages = ['Python', 'C++', 'JavaScript', 'None', 'Other'];
const skillLevels = ['Beginner', 'Intermediate', 'Advanced', 'Expert'];

const SignUpForm = () => {
  const [name, setName] = useState('');
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [currentRole, setCurrentRole] = useState('');
  const [languages, setLanguages] = useState<string[]>([]);
  const [languageSkills, setLanguageSkills] = useState<{ [key: string]: string }>({});
  const [loading, setLoading] = useState(false);

  const handleLanguageChange = (lang: string) => {
    setLanguages((prev) =>
      prev.includes(lang) ? prev.filter((l) => l !== lang) : [...prev, lang]
    );

    if (lang === 'None') {
      setLanguages(['None']);
      setLanguageSkills({});
    } else if (languages.includes('None')) {
      setLanguages((prev) => prev.filter((l) => l !== 'None'));
    }
  };

  const handleSkillLevelChange = (lang: string, level: string) => {
    setLanguageSkills((prev) => ({ ...prev, [lang]: level }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);

    try {
      const payload = {
        name,
        email,
        password,
        role: currentRole || undefined,
        languages: JSON.stringify(languages),
        languageSkills: JSON.stringify(languageSkills),
      };

      const res = await fetch(`${AUTH_API_URL}/sign-up/email`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        credentials: 'include',
        body: JSON.stringify(payload),
      });

      const text = await res.text();

      if (!res.ok) {
        alert(`Sign up failed: ${text}`);
        return;
      }

      alert('✅ Sign up successful!');
      window.location.href = '/';
    } catch (error) {
      alert('❌ Network error – Server not responding');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className={styles.formContainer}>
      <form className={styles.form} onSubmit={handleSubmit}>
        <h2>Sign Up</h2>
        <p className={styles.subtitle}>
          Join our community of learners and innovators in Physical AI & Humanoid Robotics
        </p>

        <div className={styles.inputGroup}>
          <input
            type="text"
            value={name}
            onChange={(e) => setName(e.target.value)}
            required
            disabled={loading}
            placeholder=" "
          />
          <label>Full Name</label>
        </div>

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

        <div className={styles.inputGroup}>
          <select
            value={currentRole}
            onChange={(e) => setCurrentRole(e.target.value)}
            disabled={loading}
          >
            <option value="">Select a role (optional)</option>
            {roles.map((role) => (
              <option key={role} value={role}>
                {role}
              </option>
            ))}
          </select>
          <label>Your Current Role</label>
        </div>

        <label style={{ marginBottom: '0.5rem', fontWeight: 600 }}>
          Programming Languages You Know:
        </label>

        <div className={styles.checkboxGroup}>
          {programmingLanguages.map((lang) => (
            <label key={lang} className={styles.checkboxLabel}>
              <input
                type="checkbox"
                checked={languages.includes(lang)}
                onChange={() => handleLanguageChange(lang)}
                disabled={loading}
              />
              {lang}
            </label>
          ))}
        </div>

        {languages.filter((l) => l !== 'None').map((lang) => (
          <div key={lang} className={styles.languageSkillGroup}>
            <div className={styles.inputGroup}>
              <select
                value={languageSkills[lang] || ''}
                onChange={(e) => handleSkillLevelChange(lang, e.target.value)}
                disabled={loading}
              >
                <option value="">Select skill level</option>
                {skillLevels.map((level) => (
                  <option key={level} value={level}>
                    {level}
                  </option>
                ))}
              </select>
              <label>Skill level in {lang}</label>
            </div>
          </div>
        ))}

        <button type="submit" disabled={loading}>
          {loading ? 'Creating account...' : 'Sign Up'}
        </button>
      </form>
    </div>
  );
};

export default SignUpForm;
