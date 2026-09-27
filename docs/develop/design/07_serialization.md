# 07 - Data persistence (serialization)

**Current builds** use **JSON** for many project/config blobs to simplify debugging and tool interoperability. The text below also describes a **Cereal**-based, versioned **binary** path that remains the long-term baseline and is used in parts of the stack.

The design allows **Cereal** for versioned binary archives; the on-disk project may be binary, JSON, or a mix by module.

## 1. Why Cereal?
- **Low overhead** — compact streams, fast load
- **Header-only** — no heavy codegen step
- **STL-friendly** — `std::vector`, `std::optional`, `std::map`, `std::string`, …

## 2. Versioning
Core types must carry explicit schema versions for forward migration.

### Pattern
```cpp
// 1. Declare a version
CEREAL_CLASS_VERSION(MyType, 1);

// 2. serialize with a version
template <class Archive>
void serialize(Archive& ar, std::uint32_t const version) {
    if (version == 0) {
        ar(CEREAL_NVP(old_field));
    } else {
        ar(CEREAL_NVP(old_field));
        ar(CEREAL_NVP(new_rotation_field));
        ar(CEREAL_NVP(is_valid));
    }
}
```

## 3. Notes
- **Binary portability** — Different OS/compiler ABIs can still differ; the main target is 64-bit Linux/Windows.
- **Load failures** — Often `cereal::Exception`; the UI should catch and explain version skew.
- **NVP** — Use `CEREAL_NVP` so a future switch to JSON/XML archives stays possible.
