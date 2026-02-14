#pragma once
#include "MemoryAllocators.h"

namespace WavefrontPT::Threading {
	template<typename T, typename A = Memory::ArenaWalker<T>, size_t AllocationSize = 4096>
	class ChaseLevQueue final {
		using type_ = T;
		using allocator_ = A;

		allocator_  m_Allocator;

		std::atomic<size_t> m_Top;
		std::atomic<size_t> m_Bottom;

		size_t m_Capacity;
		size_t m_Mask;

		const T* m_UnderlyingBuffer;

	public:
		ChaseLevQueue() : m_Top(0), m_Bottom(0),
			m_Capacity(64), m_Mask(0), m_UnderlyingBuffer(m_Allocator.reserve(4096 * 1024)) {
		}
		~ChaseLevQueue() {
			m_Allocator.release();
		}
	};
}
