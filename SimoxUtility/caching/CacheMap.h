#pragma once

#include <functional>
#include <map>
#include <sstream>

#include <SimoxUtility/error/SimoxError.h>

namespace simox::caching
{


    /**
     * @brief Wrapper for using a `std::map` or `std::unordered_map` as a data cache.
     *
     * A `CacheMap` uses a "fetch function" (`FetchFn`) to retrieve values
     * for keys that are not in the cache. It can be specified (among others)
     * in the constructor, via `setFetchFn()` or when accessing an item
     * via `get()`.
     */
    template <class KeyT, class ValueT, template <class...> class MapT = std::map>
    class CacheMap
    {
    public:
        /// A fetch function which creates an item when its not already in the cache.
        using FetchFn = std::function<ValueT(KeyT key)>;


    public:
        /**
         * @brief Construct a CacheMap without default fetch function.
         *
         * Specify the fetch function when calling `get()` or via `setFetchFn()`.
         */
        CacheMap()
        {
        }

        /**
         * @brief Construct a CacheMap with default fetch function.
         */
        CacheMap(FetchFn fetchFn) : fetchFn(fetchFn)
        {
        }

        /// Set the fetch function, optionally clearing the cache (disabled by default).
        void
        setFetchFn(FetchFn fetchFn, bool clear = false)
        {
            this->fetchFn = fetchFn;
            if (clear)
            {
                this->clear();
            }
        }

        /**
         * @brief Get a value for the given key, using the given fetch function.
         *
         * @param key The key.
         * @param fetchFn The function to use when the key is not stored in the cache.
         * @return The stored value.
         */
        ValueT&
        get(const KeyT& key, std::function<ValueT(KeyT)> fetchFn)
        {
            auto it = map.find(key);
            if (it != map.end())
            {
                return it->second;
            }
            else
            {
                return this->insert(key, fetchFn(key));
            }
        }

        /**
         * @brief Get a value for the given key, using the stored fetch function.
         * Make sure to pass set a fetch function via the constructor or `setFetchFn()`.
         *
         * @param key The key.
         * @return The stored value.
         * @throw `simox::error::SimoxError` If no fetch function was specified.
         */
        ValueT&
        get(const KeyT& key)
        {
            if (!fetchFn)
            {
                this->throwNoFetchFnError();
            }
            return this->get(key, this->fetchFn);
        }

        /**
         * @brief Get a value for the given key (const version).
         *
         * This does not change the cache, therefore accessing a non-existing key
         * will cause an exception.
         *
         * @param key The key.
         * @return The stored value if it exists.
         * @throw `simox::error::SimoxError` If the key is not known.
         */
        ValueT&
        get(const KeyT& key) const
        {
            auto it = map.find(key);
            if (it != map.end())
            {
                return it->second;
            }
            else
            {
                this->throwUnknownKeyError(key);
            }
        }

        // Explicit insert

        /// Explicitly insert a value using the stored fetch function.
        ValueT&
        insert(const KeyT& key)
        {
            if (!fetchFn)
            {
                this->throwNoFetchFnError();
            }
            return this->insert(key, this->fetchFn);
        }

        /// Explicitly insert a value.
        ValueT&
        insert(const KeyT& key, const ValueT& value)
        {
            return map.emplace(key, value).first->second;
        }

        /// Explicitly insert a value using the fetch function.
        ValueT&
        insert(const KeyT& key, std::function<ValueT(KeyT)> fetchFn)
        {
            return this->insert(key, fetchFn(key));
        }

        // Standard container functions

        void
        clear()
        {
            map.clear();
        }

        size_t
        size() const
        {
            return map.size();
        }

        bool
        empty() const
        {
            return map.empty();
        }

        bool
        contains(const KeyT& key) const
        {
            return map.count(key) > 0;
        }

        auto
        begin()
        {
            return map.begin();
        }

        auto
        begin() const
        {
            return map.begin();
        }

        auto
        end()
        {
            return map.end();
        }

        auto
        end() const
        {
            return map.end();
        }

        auto
        find(const KeyT& key)
        {
            return map.find(key);
        }

        auto
        find(const KeyT& key) const
        {
            return map.find(key);
        }


    private:
        /// @throw `simox::error::SimoxError`
        void
        throwNoFetchFnError() const
        {
            std::stringstream ss;
            ss << "No fetch function was defined in CacheMap before calling get without explicit "
                  "fetch function."
               << "\nCall `setFetchFn()` beforehand or pass the fetch function in the constructor.";
            throw error::SimoxError(ss.str());
        }

        /// @throw `simox::error::SimoxError`
        void
        throwUnknownKeyError(const KeyT& key) const
        {
            std::stringstream ss;
            // ToDo: Only do this if boost::has_left_shift<std::ostream, KeyT>?
            // Would need to include boost headers ...
            ss << "Tried to get non-existing value of key '" << key << "' from const cache.";
            throw simox::error::SimoxError(ss.str());
        }


    private:
        /// The data container.
        MapT<KeyT, ValueT> map;
        /// The fetch function (if specified).
        std::function<ValueT(KeyT)> fetchFn;
    };

} // namespace simox::caching
