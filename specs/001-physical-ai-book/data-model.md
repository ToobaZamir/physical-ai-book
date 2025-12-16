# Data Model: Physical AI & Humanoid Robotics Book

This document defines the core data entities that structure the book and its accompanying source code repository.

## 1. Book

The top-level entity representing the entire educational product.

-   **Fields**:
    -   `title`: String (e.g., "Physical AI & Humanoid Robotics – The Complete Practitioner’s Book")
    -   `version`: String (e.g., "2026 Edition")
    -   `license`: String ("CC-BY-NC-SA 4.0")
    -   `authors`: List<String>
-   **Relationships**:
    -   Has exactly 4 `Module` entities.
    -   Has exactly 5 `Appendix` entities.

## 2. Module

A logical section of the book, corresponding to a multi-week learning block.

-   **Fields**:
    -   `module_number`: Integer (1-4)
    -   `title`: String (e.g., "Module 1: The Robotic Nervous System – ROS 2 Mastery")
    -   `week_range`: String (e.g., "Weeks 1–5")
-   **Relationships**:
    -   Belongs to one `Book`.
    -   Has 2-3 `Chapter` entities.

## 3. Chapter

A single chapter within a module. This is the primary unit of content.

-   **Fields**:
    -   `chapter_number`: String (e.g., "1.1", "4.3")
    -   `title`: String (e.g., "Why ROS 2 Is the Only Serious Choice in 2025")
    -   `content`: Markdown/LaTeX source.
-   **Relationships**:
    -   Belongs to one `Module`.
    -   Has many `CodeExample` entities.
    -   Has many `Diagram` entities.

## 4. Appendix

A supplementary section at the end of the book.

-   **Fields**:
    -   `appendix_letter`: String ("A"-"E")
    -   `title`: String (e.g., "Complete Economy Jetson Kit BOM + wiring diagrams")
-   **Relationships**:
    -   Belongs to one `Book`.

## 5. Code Example

A runnable piece of code associated with a chapter.

-   **Fields**:
    -   `path`: String (Path to the code in the `/code` directory of the repository).
    -   `description`: String (What the code demonstrates).
    -   `qr_code`: PNG (Image file linking to the exact commit and path on GitHub).
-   **Relationships**:
    -   Belongs to one `Chapter`.

## 6. Diagram

A visual illustration for the book.

-   **Fields**:
    -   `path`: String (Path to the rendered image in `book/assets/images`).
    -   `source_path`: String (Path to the `.drawio` or `.fig` source file in `book/assets/diagrams`).
    -   `caption`: String.
-   **Relationships**:
    -   Belongs to one `Chapter`.
