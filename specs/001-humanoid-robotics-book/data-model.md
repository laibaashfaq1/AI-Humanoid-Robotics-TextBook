# Data Model: Physical AI & Humanoid Robotics Book Content

**Branch**: `001-humanoid-robotics-book` | **Date**: 2025-12-04
**Input**: Feature specification from `specs/001-humanoid-robotics-book/spec.md`
**Purpose**: Describes the structural entities of the book's content.

## Entities

### Module
*   **Description**: A top-level organizational unit of the book, grouping related chapters.
*   **Attributes**:
    *   `title` (string): Human-readable title of the module.
    *   `slug` (string): URL-friendly identifier for the module (e.g., `ros2-fundamentals`).
    *   `order` (integer): Numeric order for display in the book's navigation.
    *   `chapters` (list of Chapter): Contains a list of Chapter entities belonging to this module.
*   **Relationships**: Has many Chapters.

### Chapter
*   **Description**: A subsection within a Module, containing the core educational content.
*   **Attributes**:
    *   `title` (string): Human-readable title of the chapter.
    *   `slug` (string): URL-friendly identifier for the chapter.
    *   `order` (integer): Numeric order for display within its module.
    *   `content` (string, MDX): The main body of the chapter, including text, Markdown, MDX components, and embedded Code Snippets.
    *   `word_count` (integer): Number of words in the chapter content (constraint: 800-1500 words).
    *   `references` (list of Reference): Citations used within the chapter.
*   **Relationships**: Belongs to one Module, has many Code Snippets, has many References.

### Code Snippet
*   **Description**: A runnable piece of code embedded within a Chapter, demonstrating a concept or solution.
*   **Attributes**:
    *   `id` (string): Unique identifier for the snippet (e.g., `ros2-publisher-example`).
    *   `language` (string): Programming language (e.g., Python, C#).
    *   `path` (string): Relative path to the actual code file within the `code-examples/` directory.
    *   `content` (string): The raw code for the snippet.
    *   `description` (string): Brief explanation of what the snippet does.
    *   `dependencies` (list of string): Specific software dependencies required to run the snippet (e.g., `rclpy`, `numpy`).
    *   `platform_requirements` (string): OS/hardware requirements if specific to this snippet.
    *   `license` (string): License under which the code is distributed (Apache 2.0).
*   **Relationships**: Belongs to one Chapter.

### Reference
*   **Description**: A citation to an authoritative source used to back claims in the book.
*   **Attributes**:
    *   `id` (string): Unique identifier for the reference.
    *   `citation_text` (string): Formatted citation (e.g., APA style).
    *   `url` (string, optional): URL to the source.
    *   `type` (string): Type of source (e.g., "Official Doc", "Academic Paper", "Blog Post").
*   **Relationships**: Belongs to one Chapter.

### Asset
*   **Description**: Images, diagrams, videos, or other media embedded in chapters.
*   **Attributes**:
    *   `filename` (string): File name (e.g., `ros2-graph.png`).
    *   `path` (string): Relative path to the asset within `book/static/` or `book/docs/assets/`.
    *   `alt_text` (string): Alternative text for accessibility.
    *   `caption` (string, optional): Displayed caption.
*   **Relationships**: Belongs to one Chapter.
