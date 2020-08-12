#pragma once

//https://edmundv.home.xs4all.nl/blog/2014/07/16/reversed-range-based-for-loop/
template<class T> class ReverseIterator {
private:
    T &m_container;
public:
    ReverseIterator(T &container) : m_container(container) {}
    typename T::reverse_iterator begin() {return m_container.rbegin();}
    typename T::reverse_iterator end() {return m_container.rend();}
};
