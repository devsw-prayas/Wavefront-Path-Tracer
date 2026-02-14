#pragma once

namespace WavefrontPT::Memory {
	enum class MemoryOperation final : uint32_t {
		Reserve, Commit, Release, Free
	};

	static constexpr size_t PAGE_FILE = 4096;

	constexpr size_t alignToPage(size_t v_Size) {
		return (v_Size + PAGE_FILE - 1) & ~(PAGE_FILE - 1);
	}

	void* alloc(void* p_Memory, size_t v_Bytes, MemoryOperation v_Ops);
	bool free(void* p_Memory, size_t v_Bytes, MemoryOperation v_Ops);
}
