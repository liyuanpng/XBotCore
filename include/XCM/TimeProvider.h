/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Arturo Laurenzi, Luca Muratore
 * email:  arturo.laurenzi@iit.it, luca.muratore@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/

#ifndef __XBOT_TIME_PROVIDER_H__
#define __XBOT_TIME_PROVIDER_H__

#include <memory>

namespace XBot {

class TimeProvider {

public:

    typedef std::shared_ptr<TimeProvider> Ptr;

    virtual double get_time() const = 0;

};

template <typename Callable>
class TimeProviderFunction : public TimeProvider {

public:

    TimeProviderFunction(Callable f): f(f) {}

    virtual double get_time() const{
        return f();
    }

private:

    Callable f;
};


class SimpleTimeProvider : public TimeProvider {

public:

    void set_time(double time){ _time = time; }

    virtual double get_time() const{
        return _time;
    }

private:

    double _time;
};


}

#endif // __XBOT_TIME_PROVIDER_H__