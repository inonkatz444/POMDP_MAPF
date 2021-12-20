package pomdp.utilities;

import java.io.Serializable;
import java.util.Map;
import java.util.Objects;

public class Pair<K,V> implements Map.Entry<K,V>, Serializable{
	protected K m_first;
	protected V m_second;
	
	public Pair( K first, V second ){
		m_first = first;
		m_second = second;
	}
	
	public Pair() {
		this( null, null );
	}

	public K first(){
		return m_first;
	}
	public V second(){
		return m_second;
	}
	public String toString(){
		return "<" + m_first + ", " + m_second + ">";
	}

	public K getKey() {
		return first();
	}

	public V getValue() {
		// TODO Auto-generated method stub
		return second();
	}

	public V setValue(V value) {
		V vPrevious = second();
		m_second = value;
		return vPrevious;
	}

	public void setFirst( K o ) {
		m_first = o;
	}
	public void setSecond( V o ) {
		m_second = o;	
	}

	@Override
	public boolean equals(Object o) {
		if (this == o) return true;
		if (o == null || getClass() != o.getClass()) return false;
		Pair<?, ?> pair = (Pair<?, ?>) o;
		return Objects.equals(m_first, pair.m_first) && Objects.equals(m_second, pair.m_second);
	}

	@Override
	public int hashCode() {
		return Objects.hash(m_first, m_second);
	}
}
