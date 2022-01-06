#include <kvs/Math>
#include <kvs/Stat>
#include <kvs/Quaternion>

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

    /*
    {
        const auto vec = position_crr - position_prv;
        for( size_t i = 0; i < point_interval - 1; i++ )
        {
            //const kvs::Vec3 a( i / point_interval, i / point_interval, i / point_interval ); 
            const auto xyz = position_prv + vec * i / point_interval;
            const float x = xyz[0];
            const float y = xyz[1];
            const float z = xyz[2];
            float r, t, p;
            r = sqrt( xyz.dot( xyz ) );
            if( r == 0 )
            {
                t = 0;
                p = 0;
            }
            else
            {
                t = acos( y / r );
                if( ( x == 0 ) && ( z == 0 ) )
                {
                    p = 0;
                }
                else{
                    if( x >= 0 )
                    {
                        p = acos( z / sqrt( z * z + x * x ) );
                    }
                    else
                    {
                        p = -1 * acos( z / sqrt( z * z + x * x ) );
                    }
                }
            }
            if( p < 0 )
            {
                p += 2 * kvs::Math::pi;
            }
            const kvs::Vec3 rtp( r, t, p );
            path.add( { dir, xyz, rtp, l } );
        }
    }
    */

    {
        if( position_prv == position_crr )
        {
            const auto xyz = position_prv;
            const float x = xyz[0];
            const float y = xyz[1];
            const float z = xyz[2];
            float r, t, p;
            r = sqrt( xyz.dot( xyz ) );
            if( r == 0 )
            {
                t = 0;
                p = 0;
            }
            else{
                t = acos( y / r );
                if( ( x == 0 ) && ( z == 0 ) )
                {
                    p = 0;
                }
                else{
                    if( x >= 0 )
                    {
                        p = acos( z / sqrt( z * z + x * x ) );
                    }
                    else
                    {
                        p = -1 * acos( z / sqrt( z * z + x * x ) );
                    }
                }
            }
            if( p < 0 )
            {
                p += 2 * kvs::Math::pi;
            }
            const kvs::Vec3 rtp( r, t, p );

            kvs::Vec3 pp;
            if( rtp[1] > kvs::Math::pi / 2 ){
                pp = rtp - kvs::Vec3( { 0, kvs::Math::pi / 2, 0 } );
            }
            else{
                pp = rtp + kvs::Vec3( { 0, kvs::Math::pi / 2, 0 } );
            }
            const float pp_x = pp[0] * std::sin( pp[1] ) * std::sin( pp[2] );
            const float pp_y = pp[0] * std::cos( pp[1] );
            const float pp_z = pp[0] * std::sin( pp[1] ) * std::cos( pp[2] );
            kvs::Vec3 u;
            if( rtp[1] > kvs::Math::pi / 2 ){
                u = kvs::Vec3( { pp_x, pp_y, pp_z } );
            }
            else{
                u = -1 * kvs::Vec3( { pp_x, pp_y, pp_z } );
            }

            for( size_t i = 0; i < point_interval - 1; i++ )
            {
                path.add( { dir, xyz, u, l } );
            }
        }
        else{
            const auto q_rot = kvs::Quaternion::RotationQuaternion( position_prv, position_crr );
            const auto axis = q_rot.axis();
            const auto angle = q_rot.angle();
            const auto da = angle / point_interval;
            
            for( size_t i = 1; i < point_interval; i++ )
            {
                const auto xyz = kvs::Quaternion::Rotate( position_prv, axis, da * i );
                path.add( { dir, xyz, axis, l } );
            }
        }
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
                m_path = createPath( m_previous_position, m_current_position, m_interval );
                const auto length = m_data_queue.size();
                for( size_t i = 0; i < length; i++ )
                {
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
