import React, { type ReactNode } from 'react';
import { FaGithub, FaLinkedin } from 'react-icons/fa';
import './custom.css';

function Footer(): ReactNode {
  return (
    <div className="footer-custom">
      <h3>AI & Humanoid Robotics Book</h3>

      <div className="social-links">
        <a
          href="https://github.com/laibaashfaq1/AI-Humanoid-Robotics-TextBook"
          target="_blank"
          rel="noopener noreferrer"
        >
          <FaGithub />
        </a>
        <a
          href="https://www.linkedin.com/in/laiba-a-3a8502274?utm_source=share&utm_campaign=share_via&utm_content=profile&utm_medium=android_app"
          target="_blank"
          rel="noopener noreferrer"
        >
          <FaLinkedin />
        </a>
      </div>

      {/* --- COPYRIGHT SECTION --- */}
      <p className="copyright">
        © 2025 Humanoid Robotics Textbook. All rights reserved.
      </p>
    </div>
  );
}

export default React.memo(Footer);
