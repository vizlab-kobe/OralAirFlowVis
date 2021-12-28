#include <kvs/Math>
#include <kvs/Stat>


namespace
{

inline InSituVis::Viewpoint createPath(
    const kvs::Vec3& position_prv,
    const kvs::Vec3& position_crr,
    const size_t point_interval )
{
    using Viewpoint = InSituVis::Viewpoint;
    const kvs::Vec3 l = { 0, 0, 0 };
    auto dir = Viewpoint::Direction::Uni;
    auto path = Viewpoint();
    const auto vec = position_crr - position_prv;
    for( size_t i = 0; i < point_interval - 1; i++ ){
        //const kvs::Vec3 a( i / point_interval, i / point_interval, i / point_interval ); 
        const auto xyz = position_prv + vec * i / point_interval;
        const float x = xyz[0];
        const float y = xyz[1];
        const float z = xyz[2];
        float r, t, p;
        r = sqrt( xyz.dot( xyz ) );
        if( r == 0 ){
            t = 0;
            p = 0;
        }
        else{
            t = acos( y / r );
            if( ( x == 0 ) && ( z == 0 ) ){
                p = 0;
            }
            else{
                if( x >= 0 ){
                    p = acos( z / sqrt( z * z + x * x ) );
                }
                else{
                    p = -1 * acos( z / sqrt( z * z + x * x ) );
                }
            }
        }
        if( p < 0 ){
            p += 2 * kvs::Math::pi;
        }
        const kvs::Vec3 rtp( r, t, p );
        path.add( { dir, xyz, rtp, l } );
    }

    return path;
}

}

namespace local
{

inline void AdaptiveTimestepController::push( const Data& data )
{
    if ( m_data_queue.empty() && m_previous_data.empty() )
    {
        // Initial step.
        m_previous_position = this->calcProcess( data );
        //this->calcProcess( data );
        m_previous_data = data;
    }
    else
    {
        if ( this->isCacheEnabled() )
        {
            if( m_data_queue.size() < ( m_interval - 1 ) ){
                m_data_queue.push( data );
            }
            else{
                m_current_position = this->calcProcess( data );
                setPath( createPath( m_previous_position, m_current_position, m_interval ) );
                const auto length = m_data_queue.size();
                for( size_t i = 0; i < length; i++ ){
                    const auto data_front = m_data_queue.front();
                    this->process( data_front, m_path, i );
                    m_data_queue.pop();
                }
                m_previous_position = m_current_position;
                DataQueue().swap( m_data_queue );
                m_path.clear();
            }
        }
    }
}

} // end of namespace InSituVis
