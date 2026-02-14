#pragma once

namespace WavefrontPT::Memory {
	template<typename T, typename Derived>
	class AdvancingAllocator {
	public:
		T* reserve(size_t v_PageSize) {
			return static_cast<Derived*>(this)->reserve(v_PageSize);
		}

		T* commitForward(size_t v_Count) {
			return static_cast<Derived*>(this)->commitForward(v_Count);
		}

		bool decommit(size_t v_Count) {
			return static_cast<Derived*>(this)->decommit(v_Count);
		}

		bool release() {
			return static_cast<Derived*>(this)->release();
		}

		template<typename ...Params>
		T* constructAndAdvance(T* p_Memory, Params&&...u_Params) {
			return static_cast<Derived*>(this)->constructAndAdvance(p_Memory, std::forward<Params>(u_Params)...);
		}
	};

	template<typename T, typename Derived>
	class ArrayAllocator
	{
	public:
		T* reserve(size_t v_Count) {
			return static_cast<Derived*>(this)->reserve(v_Count);
		}

		bool commit(size_t v_Count) {
			return static_cast<Derived*>(this)->commit(v_Count);
		}

		T* allocate(size_t v_Count) {
			return static_cast<Derived*>(this)->allocate(v_Count);
		}

		T& at(size_t v_Index) {
			return static_cast<Derived*>(this)->at(v_Index);
		}

		const T& at(size_t v_Index) const {
			return static_cast<const Derived*>(this)->at(v_Index);
		}

		T* data() {
			return static_cast<Derived*>(this)->data();
		}

		const T* data() const {
			return static_cast<const Derived*>(this)->data();
		}

		bool release() {
			return static_cast<Derived*>(this)->release();
		}
	};
}
