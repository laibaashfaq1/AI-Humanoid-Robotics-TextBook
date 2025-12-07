import * as React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';

// Import custom CSS for styling
import '../css/custom.css';

export default function Home() {
  const { siteConfig } = useDocusaurusContext();

  return (
    <Layout
      title="Home"
      description="The definitive guide to Physical AI and Humanoid Robotics"
    >
      <div className="homepage-light">
        <main className="front-page-main-light">
          <header className="front-page-header-light">

            {/* HERO SECTION */}
            <div className="hero-section-light">
              <h1 className="main-heading-light">
                Build Intelligent Machines That
                <span className="gradient-text-light">  Move, Sense, and Think</span>
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
