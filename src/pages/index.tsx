import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

// Import custom CSS for styling
import '../css/custom.css';

// SVG Icon for the book logo
const BookIcon = () => (
  <svg
    xmlns="http://www.w3.org/2000/svg"
    className="h-6 w-6 mr-2 inline-block book-icon-svg"
    fill="none"
    viewBox="0 0 24 24"
    stroke="currentColor"
    strokeWidth={2}
  >
    <path
      strokeLinecap="round"
      strokeLinejoin="round"
      d="M12 6.253v13m0-13C10.832 5.477 9.246 5 7.5 5S4.168 5.477 3 6.253v13C4.168 18.477 5.754 18 7.5 18s3.332.477 4.5 1.253m0-13C13.168 5.477 14.754 5 16.5 5c1.747 0 3.332.477 4.5 1.253v13C19.832 18.477 18.247 18 16.5 18c-1.746 0-3.332.477-4.5 1.253"
    />
  </svg>
);


export default function Home(): JSX.Element {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title="Home"
      description="The definitive guide to Physical AI and Humanoid Robotics"
    >
      <div className="homepage-light"> {/* New wrapper class for light theme */}
        <main className="front-page-main-light">
          <header className="front-page-header-light">
            
            
            <div className="hero-section-light">
              <h1 className="main-heading-light">
                Engineer the Next Generation of
                <span className="gradient-text-light"> Thinking Machines</span>
              </h1>
              
              <p className="subheading-light">
                A comprehensive guide to the mechanics, sensors, locomotion, and AI that power modern humanoid robots. From first principles to advanced, real-world applications.
              </p>
              
              <div className="cta-button-container-light">
                <Link
                  className="cta-button-light"
                  to="/docs/module1/chapter1"
                >
                  Get the Textbook →
                </Link>
              </div>
            </div>
          </header>
        </main>
      </div>
    </Layout>
  );
}