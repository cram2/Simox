#pragma once

#if __has_include(<span>)
#include <span>
#else
//https://raw.githubusercontent.com/gcc-mirror/gcc/582c57a17eaf02e90492145cd7217bda5499076b/libstdc%2B%2B-v3/include/std/span
// Components for manipulating non-owning sequences of objects -*- C++ -*-

// Copyright (C) 2019 Free Software Foundation, Inc.
//
// This file is part of the GNU ISO C++ Library.  This library is free
// software; you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the
// Free Software Foundation; either version 3, or (at your option)
// any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// Under Section 7 of GPL version 3, you are granted additional
// permissions described in the GCC Runtime Library Exception, version
// 3.1, as published by the Free Software Foundation.

// You should have received a copy of the GNU General Public License and
// a copy of the GCC Runtime Library Exception along with this program;
// see the files COPYING3 and COPYING.RUNTIME respectively.  If not, see
// <http://www.gnu.org/licenses/>.

/** @file span
 *  This is a Standard C++ Library header.
 */

//
// P0122 span library
// Contributed by ThePhD
//

#ifndef _GLIBCXX_SPAN
#define _GLIBCXX_SPAN 1

#pragma GCC system_header

#if __cplusplus > 201703L

#include <array>
#include <tuple>
#include <type_traits>
#include <utility>

#include <bits/range_access.h>
#include <bits/stl_iterator.h>

namespace std _GLIBCXX_VISIBILITY(default)
{
    _GLIBCXX_BEGIN_NAMESPACE_VERSION

#define __cpp_lib_span 201902L

    inline constexpr size_t dynamic_extent = static_cast<size_t>(-1);

    template <typename _Type, size_t _Extent>
    class span;

    namespace __detail
    {
        template <typename _Tp>
        struct __is_std_span : false_type
        {
        };

        template <typename _Tp, size_t _Num>
        struct __is_std_span<span<_Tp, _Num>> : true_type
        {
        };

        template <typename _Tp>
        struct __is_std_array : false_type
        {
        };

        template <typename _Tp, size_t _Num>
        struct __is_std_array<_GLIBCXX_STD_C::array<_Tp, _Num>> : true_type
        {
        };

#ifdef _GLIBCXX_DEBUG
        template <typename _Tp, size_t _Num>
        struct __is_std_array<__debug::array<_Tp, _Num>> : true_type
        {
        };
#endif

        template <size_t _Extent>
        class __extent_storage
        {
        public:
            constexpr __extent_storage(size_t) noexcept
            {
            }

            static constexpr size_t
            _M_extent() noexcept
            {
                return _Extent;
            }
        };

        template <>
        class __extent_storage<dynamic_extent>
        {
        public:
            constexpr __extent_storage(size_t __extent) noexcept : _M_extent_value(__extent)
            {
            }

            constexpr size_t
            _M_extent() const noexcept
            {
                return this->_M_extent_value;
            }

        private:
            size_t _M_extent_value;
        };

    } // namespace __detail

    template <typename _Type, size_t _Extent = dynamic_extent>
    class span
    {
        template <size_t _Offset, size_t _Count>
        static constexpr size_t
        _S_subspan_extent()
        {
            if constexpr (_Count != dynamic_extent)
                return _Count;
            else if constexpr (extent != dynamic_extent)
                return _Extent - _Offset;
            else
                return dynamic_extent;
        }

        template <typename _Tp>
        using __is_compatible = is_convertible<_Tp (*)[], _Type (*)[]>;

        // _GLIBCXX_RESOLVE_LIB_DEFECTS
        // 3255. span's array constructor is too strict
        template <typename _Tp,
                  size_t _ArrayExtent,
                  typename = enable_if_t<_Extent == dynamic_extent || _ArrayExtent == _Extent>>
        using __is_compatible_array = __is_compatible<_Tp>;

    public:
        // member types
        using value_type = remove_cv_t<_Type>;
        using element_type = _Type;
        using index_type = size_t;
        using reference = element_type&;
        using const_reference = const element_type&;
        using pointer = _Type*;
        using const_pointer = const _Type*;
        using iterator = __gnu_cxx::__normal_iterator<pointer, span>;
        using const_iterator = __gnu_cxx::__normal_iterator<const_pointer, span>;
        using reverse_iterator = std::reverse_iterator<iterator>;
        using const_reverse_iterator = std::reverse_iterator<const_iterator>;
        using difference_type = ptrdiff_t;
        // Official wording has no size_type -- why??
        // using size_type = size_t;

        // member constants
        static inline constexpr size_t extent = _Extent;

        // constructors

        template <bool _DefaultConstructible = (_Extent + 1u) <= 1u,
                  enable_if_t<_DefaultConstructible>* = nullptr>
        constexpr span() noexcept : _M_extent(0), _M_ptr(nullptr)
        {
        }

        constexpr span(const span&) noexcept = default;

        template <typename _Tp,
                  size_t _ArrayExtent,
                  typename = _Require<__is_compatible_array<_Tp, _ArrayExtent>>>
        constexpr span(_Tp (&__arr)[_ArrayExtent]) noexcept :
            span(static_cast<pointer>(__arr), _ArrayExtent)
        {
        }

        template <typename _Tp,
                  size_t _ArrayExtent,
                  typename = _Require<__is_compatible_array<_Tp, _ArrayExtent>>>
        constexpr span(array<_Tp, _ArrayExtent>& __arr) noexcept :
            span(static_cast<pointer>(__arr.data()), _ArrayExtent)
        {
        }

        template <typename _Tp,
                  size_t _ArrayExtent,
                  typename = _Require<__is_compatible_array<_Tp, _ArrayExtent>>>
        constexpr span(const array<_Tp, _ArrayExtent>& __arr) noexcept :
            span(static_cast<pointer>(__arr.data()), _ArrayExtent)
        {
        }

        // NOTE: when the time comes, and P1394 -
        // range constructors for std::span - ships in
        // the standard, delete the #else block and remove
        // the conditional
        // if the paper fails, delete #if block
        // and keep the crappy #else block
        // and then cry that NB comments failed C++20...
        // but maybe for C++23?
#ifdef _GLIBCXX_P1394
    private:
        // FIXME: use std::iter_reference_t
        template <typename _Iterator>
        using iter_reference_t = decltype(*std::declval<_Iterator&>());
        // FIXME: use std::ranges::iterator_t
        // N.B. constraint is needed to prevent a cycle when __adl_begin finds
        // begin(span) which does overload resolution on span(Range&&).
        template <typename _Rng,
                  typename _Rng2 = remove_cvref_t<_Rng>,
                  typename = enable_if_t<!__detail::__is_std_span<_Rng2>::value>>
        using iterator_t = decltype(std::__adl_begin(std::declval<_Rng&>()));
        // FIXME: use std::iter_value_t
        template <typename _Iter>
        using iter_value_t = typename iterator_traits<_Iter>::value_type;
        // FIXME: use std::derived_from concept
        template <typename _Derived, typename _Base>
        using derived_from =
            __and_<is_base_of<_Base, _Derived>,
                   is_convertible<const volatile _Derived*, const volatile _Base*>>;
        // FIXME: require contiguous_iterator<_Iterator>
        template <typename _Iter,
                  typename _Ref = iter_reference_t<_Iter>,
                  typename _Traits = iterator_traits<_Iter>,
                  typename _Tag = typename _Traits::iterator_category>
        using __is_compatible_iterator = __and_<derived_from<_Tag, random_access_iterator_tag>,
                                                is_lvalue_reference<_Ref>,
                                                is_same<iter_value_t<_Iter>, remove_cvref_t<_Ref>>,
                                                __is_compatible<remove_reference_t<_Ref>>>;

        template <typename _Range>
        using __is_compatible_range = __is_compatible_iterator<iterator_t<_Range>>;

    public:
        template <typename _Range,
                  typename = _Require<bool_constant<_Extent == dynamic_extent>,
                                      __not_<__detail::__is_std_span<remove_cvref_t<_Range>>>,
                                      __not_<__detail::__is_std_array<remove_cvref_t<_Range>>>,
                                      __not_<is_array<remove_reference_t<_Range>>>,
                                      __is_compatible_range<_Range>>,
                  typename = decltype(std::__adl_data(std::declval<_Range&>()))>
        constexpr span(_Range&& __range) noexcept(
            noexcept(::std::__adl_data(__range)) && noexcept(::std::__adl_size(__range))) :
            span(::std::__adl_data(__range), ::std::__adl_size(__range))
        {
        }

        template <typename _ContiguousIterator,
                  typename _Sentinel,
                  typename = _Require<__not_<is_convertible<_Sentinel, index_type>>,
                                      __is_compatible_iterator<_ContiguousIterator>>>
        constexpr span(_ContiguousIterator __first, _Sentinel __last) :
            _M_extent(static_cast<index_type>(__last - __first)), _M_ptr(std::to_address(__first))
        {
            if (_Extent != dynamic_extent)
                __glibcxx_assert((__last - __first) == _Extent);
        }

        template <typename _ContiguousIterator,
                  typename = _Require<__is_compatible_iterator<_ContiguousIterator>>>
        constexpr span(_ContiguousIterator __first,
                       index_type __count) noexcept(noexcept(std::to_address(__first))) :
            _M_extent(__count), _M_ptr(std::to_address(__first))
        {
            __glibcxx_assert(_Extent == dynamic_extent || __count == _Extent);
        }
#else
    private:
        template <typename _Container,
                  typename _DataT = decltype(std::data(std::declval<_Container&>())),
                  typename _SizeT = decltype(std::size(std::declval<_Container&>()))>
        using __is_compatible_container = __is_compatible<remove_pointer_t<_DataT>>;

    public:
        template <typename _Container,
                  typename = _Require<bool_constant<_Extent == dynamic_extent>,
                                      __not_<__detail::__is_std_span<remove_cv_t<_Container>>>,
                                      __not_<__detail::__is_std_array<remove_cv_t<_Container>>>,
                                      __not_<is_array<_Container>>,
                                      __is_compatible_container<_Container>>>
        constexpr span(_Container& __cont) noexcept(
            noexcept(std::data(__cont)) && noexcept(std::size(__cont))) :
            _M_extent(std::size(__cont)), _M_ptr(std::data(__cont))
        {
        }

        template <typename _Container,
                  typename = _Require<bool_constant<_Extent == dynamic_extent>,
                                      __not_<__detail::__is_std_span<remove_cv_t<_Container>>>,
                                      __not_<__detail::__is_std_array<remove_cv_t<_Container>>>,
                                      __not_<is_array<_Container>>,
                                      __is_compatible_container<const _Container>>>
        constexpr span(const _Container& __cont) noexcept(
            noexcept(std::data(__cont)) && noexcept(std::size(__cont))) :
            _M_extent(std::size(__cont)), _M_ptr(std::data(__cont))
        {
        }

        constexpr span(pointer __first, index_type __count) noexcept :
            _M_extent(__count), _M_ptr(__first)
        {
            __glibcxx_assert(_Extent == dynamic_extent || __count == _Extent);
        }

        constexpr span(pointer __first, pointer __last) noexcept :
            span(__first, static_cast<index_type>(__last - __first))
        {
        }
#endif // P1394

        template <
            typename _OType,
            size_t _OExtent,
            typename = _Require<__bool_constant<_Extent == dynamic_extent || _Extent == _OExtent>,
                                is_convertible<_OType (*)[], _Type (*)[]>>>
        constexpr span(const span<_OType, _OExtent>& __s) noexcept :
            _M_extent(__s.size()), _M_ptr(__s.data())
        {
        }

        // assignment

        constexpr span& operator=(const span&) noexcept = default;

        // observers

        constexpr index_type
        size() const noexcept
        {
            return this->_M_extent._M_extent();
        }

        constexpr index_type
        size_bytes() const noexcept
        {
            return this->_M_extent._M_extent() * sizeof(element_type);
        }

        [[nodiscard]] constexpr bool
        empty() const noexcept
        {
            return size() == 0;
        }

        // element access

        constexpr reference
        front() const noexcept
        {
            static_assert(extent != 0);
            __glibcxx_assert(!empty());
            return *this->_M_ptr;
        }

        constexpr reference
        back() const noexcept
        {
            static_assert(extent != 0);
            __glibcxx_assert(!empty());
            return *(this->_M_ptr + (size() - 1));
        }

        constexpr reference
        operator[](index_type __idx) const noexcept
        {
            static_assert(extent != 0);
            __glibcxx_assert(__idx < size());
            return *(this->_M_ptr + __idx);
        }

        constexpr pointer
        data() const noexcept
        {
            return this->_M_ptr;
        }

        // iterator support

        constexpr iterator
        begin() const noexcept
        {
            return iterator(this->_M_ptr);
        }

        constexpr const_iterator
        cbegin() const noexcept
        {
            return const_iterator(this->_M_ptr);
        }

        constexpr iterator
        end() const noexcept
        {
            return iterator(this->_M_ptr + this->size());
        }

        constexpr const_iterator
        cend() const noexcept
        {
            return const_iterator(this->_M_ptr + this->size());
        }

        constexpr reverse_iterator
        rbegin() const noexcept
        {
            return reverse_iterator(this->end());
        }

        constexpr const_reverse_iterator
        crbegin() const noexcept
        {
            return const_reverse_iterator(this->cend());
        }

        constexpr reverse_iterator
        rend() const noexcept
        {
            return reverse_iterator(this->begin());
        }

        constexpr const_reverse_iterator
        crend() const noexcept
        {
            return const_reverse_iterator(this->cbegin());
        }

        // subviews

        template <size_t _Count>
        constexpr span<element_type, _Count>
        first() const noexcept
        {
            if constexpr (_Extent == dynamic_extent)
                __glibcxx_assert(_Count <= size());
            else
                static_assert(_Count <= extent);
            return {this->data(), _Count};
        }

        constexpr span<element_type, dynamic_extent>
        first(index_type __count) const noexcept
        {
            __glibcxx_assert(__count <= size());
            return {this->data(), __count};
        }

        template <size_t _Count>
        constexpr span<element_type, _Count>
        last() const noexcept
        {
            if constexpr (_Extent == dynamic_extent)
                __glibcxx_assert(_Count <= size());
            else
                static_assert(_Count <= extent);
            return {this->data() + (this->size() - _Count), _Count};
        }

        constexpr span<element_type, dynamic_extent>
        last(index_type __count) const noexcept
        {
            __glibcxx_assert(__count <= size());
            return {this->data() + (this->size() - __count), __count};
        }

        template <size_t _Offset, size_t _Count = dynamic_extent>
        constexpr auto
        subspan() const noexcept -> span<element_type, _S_subspan_extent<_Offset, _Count>()>
        {
            if constexpr (_Extent == dynamic_extent)
                __glibcxx_assert(_Offset <= size());
            else
                static_assert(_Offset <= extent);

            if constexpr (_Count == dynamic_extent)
                return {this->data() + _Offset, this->size() - _Offset};
            else
            {
                if constexpr (_Extent == dynamic_extent)
                {
                    __glibcxx_assert(_Count <= size());
                    __glibcxx_assert(_Count <= (size() - _Offset));
                }
                else
                {
                    static_assert(_Count <= extent);
                    static_assert(_Count <= (extent - _Offset));
                }
                return {this->data() + _Offset, _Count};
            }
        }

        constexpr span<element_type, dynamic_extent>
        subspan(index_type __offset, index_type __count = dynamic_extent) const noexcept
        {
            __glibcxx_assert(__offset <= size());
            if (__count == dynamic_extent)
                __count = this->size() - __offset;
            else
            {
                __glibcxx_assert(__count <= size());
                __glibcxx_assert(__offset + __count <= size());
            }
            return {this->data() + __offset, __count};
        }

        // observers: range helpers

        friend constexpr iterator
        begin(span __sp) noexcept
        {
            return __sp.begin();
        }

        friend constexpr iterator
        end(span __sp) noexcept
        {
            return __sp.end();
        }

    private:
        [[no_unique_address]] __detail::__extent_storage<extent> _M_extent;
        pointer _M_ptr;
    };

    // deduction guides
    template <typename _Type, size_t _ArrayExtent>
    span(_Type (&)[_ArrayExtent]) -> span<_Type, _ArrayExtent>;

    template <typename _Type, size_t _ArrayExtent>
    span(array<_Type, _ArrayExtent>&) -> span<_Type, _ArrayExtent>;

    template <typename _Type, size_t _ArrayExtent>
    span(const array<_Type, _ArrayExtent>&) -> span<const _Type, _ArrayExtent>;

#ifdef _GLIBCXX_P1394

    template <typename _ContiguousIterator, typename _Sentinel>
    span(_ContiguousIterator, _Sentinel)
        -> span<remove_reference_t<typename iterator_traits<_ContiguousIterator>::reference>>;

    template <typename _Range>
    span(_Range&&) -> span<remove_reference_t<typename iterator_traits<
        decltype(std::__adl_begin(::std::declval<_Range&>()))>::reference>>;

#else

    template <typename _Container>
    span(_Container&) -> span<typename _Container::value_type>;

    template <typename _Container>
    span(const _Container&) -> span<const typename _Container::value_type>;

#endif // P1394

    template <typename _Type, size_t _Extent>
    inline span<const byte, _Extent == dynamic_extent ? dynamic_extent : _Extent * sizeof(_Type)>
    as_bytes(span<_Type, _Extent> __sp) noexcept
    {
        return {reinterpret_cast<const byte*>(__sp.data()), __sp.size_bytes()};
    }

    template <typename _Type, size_t _Extent>
    inline span<byte, _Extent == dynamic_extent ? dynamic_extent : _Extent * sizeof(_Type)>
    as_writable_bytes(span<_Type, _Extent> __sp) noexcept
    {
        return {reinterpret_cast<byte*>(__sp.data()), __sp.size_bytes()};
    }

    // tuple helpers
    template <size_t _Index, typename _Type, size_t _Extent>
    constexpr _Type&
    get(span<_Type, _Extent> __sp) noexcept
    {
        static_assert(_Extent != dynamic_extent && _Index < _Extent,
                      "get<I> can only be used with a span of non-dynamic (fixed) extent");
        return __sp[_Index];
    }

    template <typename _Type, size_t _Extent>
    struct tuple_size<span<_Type, _Extent>> : public integral_constant<size_t, _Extent>
    {
        static_assert(_Extent != dynamic_extent,
                      "tuple_size can only "
                      "be used with a span of non-dynamic (fixed) extent");
    };

    template <size_t _Index, typename _Type, size_t _Extent>
    struct tuple_element<_Index, span<_Type, _Extent>>
    {
        static_assert(_Extent != dynamic_extent,
                      "tuple_element can only "
                      "be used with a span of non-dynamic (fixed) extent");
        static_assert(_Index < _Extent, "Index is less than Extent");
        using type = _Type;
    };

    _GLIBCXX_END_NAMESPACE_VERSION
} // namespace std _GLIBCXX_VISIBILITY(default)

#endif // C++20
#endif // _GLIBCXX_SPAN

#endif // __has_include ( < span > )
