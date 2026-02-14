#include <Core.h>
#include <Memory.h>

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>

namespace WavefrontPT::Memory {
	void* alloc(void* p_Memory, size_t v_Bytes, MemoryOperation v_Ops) {
		void* mem = nullptr;
		switch (v_Ops) {
		case MemoryOperation::Reserve:
			mem = VirtualAlloc(nullptr, v_Bytes, MEM_RESERVE, PAGE_NOACCESS);
			if (!mem) return nullptr;
			return mem;
		case MemoryOperation::Commit:
			mem = VirtualAlloc(p_Memory, v_Bytes, MEM_COMMIT, PAGE_READWRITE);
			if (!mem) return nullptr;
			return mem;
		case MemoryOperation::Release:	WF_FALLTHROUGH
		case MemoryOperation::Free:		return nullptr;
		}
		WF_UNREACHABLE();
	}

	bool free(void* p_Memory, size_t v_Bytes, MemoryOperation v_Ops) {
		if (!p_Memory) return false;
		switch (v_Ops) {
		case MemoryOperation::Free: 
			if (VirtualFree(p_Memory, v_Bytes, MEM_DECOMMIT)) return true;
			return false;
		case MemoryOperation::Release:
			if (VirtualFree(p_Memory, 0, MEM_RELEASE)) return true;
			return false;
		case MemoryOperation::Commit: WF_FALLTHROUGH
		case MemoryOperation::Reserve: return false;
		}
		WF_UNREACHABLE();
	}
}