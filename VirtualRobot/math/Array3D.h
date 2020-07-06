/**
 * This file is part of Simox.
 *
 * Simox is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Simox is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @author     Simon Ottenhaus (simon dot ottenhaus at kit dot edu)
 * @copyright  2018 Simon Ottenhaus
 *             GNU Lesser General Public License
 */

#pragma once

#include "MathForwardDefinitions.h"

#include "Index3.h"


namespace math
{
    template<class T>
    class Array3D
    {
    public:
        Array3D(int size) :
            size(size)
        {
            data = std::shared_ptr<std::vector<T>>(new std::vector<T>);
            data->resize(size*size*size);
        }

        T Get(Index3 index)
        {
            return data->at(index.X() +
                    size * index.Y() +
                    size * size * index.Z());
        }

        T Get(int i, int j, int k)
        {
            return data->at(i +
                    size * j +
                    size * size * k );

        }

        void Set(Index3 index, T value)
        {
            data->at(             index.X() +
                    size * index.Y() +
                    size * size * index.Z()) = value;
        }
        void Set(int i, int j, int k, T value)
        {
            data->at(            i +
                    size * j +
                    size * size * k )= value;
        }

    private:
        std::shared_ptr<std::vector<T>> data;
        int size;
    };
}

