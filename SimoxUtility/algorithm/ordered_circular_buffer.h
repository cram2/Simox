#pragma once

#include <algorithm>

#include <boost/circular_buffer.hpp>

namespace simox
{
    template <typename KeyT_, typename ElementT_>
    class OrderedCircularBuffer
    {
    private:
        using KeyT = KeyT_;
        using ElementT = ElementT_;

        struct Entry
        {
            KeyT first; // key
            ElementT second; // element
        };

        struct EntryKeyCompare
        {
            bool
            operator()(Entry const& entry, KeyT const& key)
            {
                return entry.first < key;
            }
        };

        using This = OrderedCircularBuffer;

        using Container = boost::circular_buffer<Entry>;
        Container storage;

    public:
        // Container typedefs
        using value_type = typename Container::value_type;
        using reference = typename Container::reference;
        using const_reference = typename Container::const_reference;
        using pointer = typename Container::pointer;
        using const_pointer = typename Container::const_pointer;
        using iterator = typename Container::iterator;
        using const_iterator = typename Container::const_iterator;
        using reverse_iterator = typename Container::reverse_iterator;
        using const_reverse_iterator = typename Container::const_reverse_iterator;

        // Container interface (forwards to storage container)
        iterator
        begin()
        {
            return storage.begin();
        }

        const_iterator
        begin() const
        {
            return storage.begin();
        }

        iterator
        end()
        {
            return storage.end();
        }

        const_iterator
        end() const
        {
            return storage.end();
        }

        reverse_iterator
        rbegin()
        {
            return storage.rbegin();
        }

        const_reverse_iterator
        rbegin() const
        {
            return storage.rbegin();
        }

        reverse_iterator
        rend()
        {
            return storage.rend();
        }

        const_reverse_iterator
        rend() const
        {
            return storage.rend();
        }

        bool
        empty() const
        {
            return storage.empty();
        }

        std::size_t
        size() const
        {
            return storage.size();
        }

        void
        clear()
        {
            storage.clear();
        }

        // Map like functions (require sorted elements)
        iterator
        find(KeyT const& key)
        {
            iterator end_ = end();
            iterator found = std::lower_bound(begin(), end_, key, EntryKeyCompare());
            // We still need to check for key equality
            if (found != end_ && found->first != key)
            {
                found = end_;
            }
            return found;
        }

        const_iterator
        find(KeyT const& key) const
        {
            return const_cast<This*>(this)->find(key);
        }

        int
        count(KeyT const& key) const
        {
            const_iterator it = find(key);
            if (it == end())
            {
                return 0;
            }
            else
            {
                return 1;
            }
        }

        ElementT&
        at(KeyT const& key)
        {
            iterator found = find(key);
            if (found == end())
            {
                throw std::out_of_range("Key was not found");
            }
            return found->second;
        }

        ElementT const&
        at(KeyT const& key) const
        {
            return const_cast<This*>(this)->at(key);
        }

        // Map insert functions (need to keep order)
        std::pair<iterator, bool>
        emplace(KeyT const& key, ElementT&& element)
        {
            std::pair<iterator, bool> result;

            // We can just insert if the container is empty
            bool justInsert = empty();
            // or the key is greater then the last key
            justInsert = justInsert || key > storage.back().first;
            if (justInsert)
            {
                storage.push_back(Entry{key, std::move(element)});

                result.first = storage.rend().base();
                result.second = true;
            }
            else
            {
                // If the container is already too full, we need to remove one element
                if (storage.full())
                {
                    storage.pop_front();
                }

                // We need to find the right insertion place
                iterator insertPosition = std::lower_bound(begin(), end(), key, EntryKeyCompare());
                // The insert position must exist, since we checked that the key is smaller than the last one
                storage.insert(insertPosition, Entry{key, std::move(element)});
            }

            return result;
        }

        static OrderedCircularBuffer
        createWithMaxSize(std::size_t maxSize)
        {
            OrderedCircularBuffer result;
            result.setMaxSize(maxSize);
            return result;
        }

        void
        setMaxSize(std::size_t maxSize)
        {
            storage.set_capacity(maxSize);
        }

        std::size_t
        getMaxSize()
        {
            // The vectors size is the max capacity
            return storage.capacity();
        }
    };

} // namespace simox
