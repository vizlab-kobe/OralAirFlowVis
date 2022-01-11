#include <kvs/Math>
#include <kvs/Stat>
#include <kvs/Quaternion>

namespace
{
/*
template <typename T>
inline float Entropy(
    const size_t width,
    const size_t height,
    const kvs::ValueArray<T>& color_buffer,
    const kvs::ValueArray<T>& depth_buffer, )
{
}
*/

inline InSituVis::Viewpoint CreatePath(
    const kvs::Vec3& position_prv,
    const kvs::Vec3& upVector_prv,
    const kvs::Vec3& position_crr,
    const kvs::Vec3& upVector_crr,
    const size_t point_interval )
{
    using Viewpoint = InSituVis::SphericalViewpoint;
    const kvs::Vec3 l = { 0, 0, 0 };
    const auto dir = Viewpoint::Direction::Uni;
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
        if ( position_prv == position_crr )
        {
            for ( size_t i = 0; i < point_interval - 1; i++ )
            {
                path.add( { dir, position_prv, upVector_prv, l } );
            }
        }
        else
        {
            const auto q_rot = kvs::Quaternion::RotationQuaternion( position_prv, position_crr );
            const auto axis = q_rot.axis();
            const auto angle = q_rot.angle();
            const auto da = angle / point_interval;

            /*const auto pp_prv = position_prv + upVector_prv;
            const auto pp_crr = position_crr + upVector_crr;
            const auto pp_rot = kvs::Quaternion::RotationQuaternion( pp_prv, pp_crr );
            const auto pp_axis = pp_rot.axis();
            const auto pp_angle = pp_rot.angle();
            const auto pp_da = pp_angle / point_interval;*/

            for ( size_t i = 1; i < point_interval; i++ )
            {
                const auto xyz = kvs::Quaternion::Rotate( position_prv, axis, da * i );

                //calculate upVector
                const float x = xyz[0];
                const float y = xyz[1];
                const float z = xyz[2];
                float r, t, p;
                r = sqrt( xyz.dot( xyz ) );
                if ( r == 0 )
                {
                    t = 0;
                    p = 0;
                }
                else
                {
                    t = acos( y / r );
                    if ( ( x == 0 ) && ( z == 0 ) )
                    {
                        p = 0;
                    }
                    else
                    {
                        if ( x >= 0 )
                        {
                            p = acos( z / sqrt( z * z + x * x ) );
                        }
                        else
                        {
                            p = -1 * acos( z / sqrt( z * z + x * x ) );
                        }
                    }
                }
                if ( p < 0 )
                {
                    p += 2 * kvs::Math::pi;
                }

                auto rtp = kvs::Vec3( { r, t, p } );

                kvs::Vec3 pp;
                if ( rtp[1] > kvs::Math::pi / 2 )
                {
                    pp = rtp - kvs::Vec3( { 0, kvs::Math::pi / 2, 0 } );
                }
                else
                {
                    pp = rtp + kvs::Vec3( { 0, kvs::Math::pi / 2, 0 } );
                }
                const float pp_x = pp[0] * std::sin( pp[1] ) * std::sin( pp[2] );
                const float pp_y = pp[0] * std::cos( pp[1] );
                const float pp_z = pp[0] * std::sin( pp[1] ) * std::cos( pp[2] );
                kvs::Vec3 u;
                if ( rtp[1] > kvs::Math::pi / 2 )
                {
                    u = kvs::Vec3( { pp_x, pp_y, pp_z } );
                }
                else
                {
                    u = -1 * kvs::Vec3( { pp_x, pp_y, pp_z } );
                }

                //const auto pp = kvs::Quaternion::Rotate( pp_prv, pp_axis, pp_da * i );
                //const auto u = pp - xyz;
                path.add( { dir, xyz, u, l } );
            }
        }
    }

    return path;
}

}

namespace local
{

inline float EntropyTimestepController::ColorEntropy( const FrameBuffer& frame_buffer )
{
    float entropy = 0.0f;

    return entropy;
}

inline float EntropyTimestepController::DepthEntropy( const FrameBuffer& frame_buffer )
{
    /*
    for ( size_t i = 0; i < ( width * height ); i++ )
    {
        if ( depth_buffer[i] < 1.0f )
        {
            const size_t j = depth_buffer[i] * 256;
            histogram[j] += 1;
            n += 1;
        }
    }

    float entropy = 0.0;
    for ( size_t i = 0; i < 256; i++ )
    {
        const float p = static_cast<float>( histogram[i] ) / n;
        if( p > 0 )
        {
            entropy -= p * log( p ) / log( 2.0f );
        }
    }
    */

    kvs::ValueArray<size_t> histogram( 256 );
    histogram.fill( 0 );

    size_t n = 0;
    const auto& depth_buffer = frame_buffer.depth_buffer;
    for ( const auto depth : depth_buffer )
    {
        if ( depth < 1.0f )
        {
            const auto i = static_cast<int>( depth * 255 );
            histogram[i] += 1;
            n += 1;
        }
    }

    float entropy = 0.0f;
    const auto log2 = std::log( 2.0f );
    for ( const auto h : histogram )
    {
        const auto p = static_cast<float>( h ) / n;
        if ( p > 0.0f ) { entropy -= p * std::log( p ) / log2; }
    }

    return entropy;
}

inline void EntropyTimestepController::push( const Data& data )
{
    if ( m_data_queue.empty() && m_previous_data.empty() )
    {
        // Initial step.
        m_previous_position = this->process( data );
        m_previous_data = data;
        this->setPrvUpVector( this->crrUpVector() );
    }
    else
    {
        if ( this->isCacheEnabled() )
        {
            if ( m_data_queue.size() < ( m_interval - 1 ) )
            {
                m_data_queue.push( data );
            }
            else
            {
                m_current_position = this->process( data );

                m_path = ::CreatePath(
                    m_previous_position,
                    this->prvUpVector(),
                    m_current_position,
                    this->crrUpVector(),
                    m_interval );

                const auto length = m_data_queue.size();
                for ( size_t i = 0; i < length; i++ )
                {
                    const auto data_front = m_data_queue.front();
                    this->process( data_front, m_path, i );
                    m_data_queue.pop();
                }

                m_previous_position = m_current_position;
                this->setPrvUpVector( crrUpVector() );
                DataQueue().swap( m_data_queue );
                m_path.clear();
            }
        }
    }
}

float EntropyTimestepController::entropy( const FrameBuffer& frame_buffer )
{
    return m_entropy_function( frame_buffer );
}

} // end of namespace InSituVis
