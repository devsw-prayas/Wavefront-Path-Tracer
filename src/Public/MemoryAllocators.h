#pragma once
#include <Memory.h>
#include <Allocator.h>

namespace WavefrontPT::Memory {
	// High performance iterative arena allocator
	template<typename T>
	class alignas(64) ArenaWalker final : public AdvancingAllocator<T, ArenaWalker<T>> {
		T* m_BaseAllocation;
		T* m_AllocationEnd;
		T* m_PtrHead;
		size_t m_TotalSize;
	public:
		ArenaWalker() : m_BaseAllocation(nullptr), m_AllocationEnd(nullptr), m_PtrHead(nullptr), m_TotalSize(0) {}
		~ArenaWalker() {
			release();
		}

		ArenaWalker(const ArenaWalker&) = delete;
		ArenaWalker& operator=(const ArenaWalker&) = delete;

		ArenaWalker(ArenaWalker&&) noexcept = default;
		ArenaWalker& operator=(ArenaWalker&&) noexcept = default;

		T* reserve(size_t v_PageSize) {
			if (m_BaseAllocation) return m_BaseAllocation;
			size_t aligned = alignToPage(v_PageSize);
			m_BaseAllocation = alloc(nullptr, aligned, MemoryOperation::Reserve);
			if (!m_BaseAllocation)	return nullptr;
			m_PtrHead = m_BaseAllocation;
			uintptr_t advanceLimit = reinterpret_cast<uintptr_t>(m_BaseAllocation) + aligned;
			m_AllocationEnd = reinterpret_cast<T*>(advanceLimit);
			m_TotalSize = aligned;
			return m_BaseAllocation;
		}

		T* commitForward(size_t v_Count) {
			if (!m_BaseAllocation) return nullptr;
			size_t aligned = alignToPage(v_Count);
			uintptr_t head = reinterpret_cast<uintptr_t>(m_PtrHead);
			uintptr_t newHead = head + aligned;
			if (newHead > reinterpret_cast<uintptr_t>(m_AllocationEnd))return nullptr;
			void* pCommit = alloc(m_PtrHead, aligned, MemoryOperation::Commit);
			if (!pCommit)return nullptr;
			T* blockStart = m_PtrHead;
			m_PtrHead = reinterpret_cast<T*>(newHead);
			return blockStart;
		}

		bool decommit(size_t v_Count) {
			if (!m_BaseAllocation) return false;
			size_t aligned = alignToPage(v_Count);
			uintptr_t base = reinterpret_cast<uintptr_t>(m_BaseAllocation);
			uintptr_t head = reinterpret_cast<uintptr_t>(m_PtrHead);
			if (aligned > (head - base)) return false;
			uintptr_t newHead = head - aligned;
			if (!alloc(reinterpret_cast<T*>(newHead), aligned, MemoryOperation::Free))return false;
			m_PtrHead = reinterpret_cast<T*>(newHead);
			return true;
		}

		bool release() {
			if (!m_BaseAllocation) return true;
			bool result = alloc(m_BaseAllocation, 0, MemoryOperation::Release);
			m_BaseAllocation = nullptr;
			m_AllocationEnd = nullptr;
			m_PtrHead = nullptr;
			m_TotalSize = 0;
			return result;
		}

		template<typename ...Params>
		T* constructAndAdvance(T* p_Memory, Params&&...u_Params) {
			new (p_Memory) T(std::forward<Params>(u_Params)...);
			return p_Memory + 1;
		}
	};
}
