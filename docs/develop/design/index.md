# InsightAT Developer Guide Index

## Core Concepts
- [00_why_Insight_AT.md](00_why_Insight_AT.md) - Why InsightAT exists and its core value proposition
- [01_algorithm_sfm_philosophy.md](01_algorithm_sfm_philosophy.md) - Algorithmic philosophy and approach to SfM
- [10_introduction.md](10_introduction.md) - Introduction to InsightAT
- [11_architecture_overview.md](11_architecture_overview.md) - System architecture overview
- [12_implementation_details.md](12_implementation_details.md) - Key implementation details

## Code Quality
- [02-coding_style.md](02-coding_style.md) - Coding style guide and best practices
- [03_directory_organization.md](03_directory_organization.md) - Directory organization principles

## System Design
- [04_functional_at_toolkit.md](04_functional_at_toolkit.md) - Functional aspects of AT toolkit
- [05_cli_io_conventions.md](05_cli_io_conventions.md) - CLI I/O conventions for tools
- [07_serialization.md](07_serialization.md) - Data persistence and serialization approaches
- [08_coordinate_and_rotation.md](08_coordinate_and_rotation.md) - Coordinate systems and rotation representations
- [09_data_model.md](09_data_model.md) - Core data model and types
- [13_idc_format_spec.md](13_idc_format_spec.md) - IDC Format Specification (InsightAT Data Container)

## UI Framework
- [06_ui_framework.md](06_ui_framework.md) - UI framework overview

---

## Suggested order for agents / new contributors

1. **Algorithm & pipeline:** [01_algorithm_sfm_philosophy.md](01_algorithm_sfm_philosophy.md) → [04_functional_at_toolkit.md](04_functional_at_toolkit.md)  
2. **Code & CLI:** [02-coding_style.md](02-coding_style.md) → [05_cli_io_conventions.md](05_cli_io_conventions.md)  
3. **Numeric & frames:** [08_coordinate_and_rotation.md](08_coordinate_and_rotation.md), then [dev-notes/rotation/rotation_readme.md](../../dev-notes/rotation/rotation_readme.md) if needed  
4. **UI & persistence:** [06_ui_framework.md](06_ui_framework.md) → [07_serialization.md](07_serialization.md)  
