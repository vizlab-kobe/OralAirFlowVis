#include <kvs/ColorImage>
#include <kvs/RGBColor>

namespace local
{

namespace mpi
{

inline void EntropyControlledAdaptor::setOutputEvaluationImageEnabled(
    const bool enable,
    const bool enable_depth )
{
    m_enable_output_evaluation_image = enable;
    m_enable_output_evaluation_image_depth = enable_depth;
}

inline bool EntropyControlledAdaptor::isEntropyStep()
{
    return BaseClass::timeStep() % ( BaseClass::analysisInterval() * Controller::entropyInterval() ) == 0;
}

inline bool EntropyControlledAdaptor::dump()
{
    bool ret = true;
    if ( BaseClass::world().isRoot() )
    {
        if ( m_entr_timer.title().empty() ) { m_entr_timer.setTitle( "Ent time" ); }
        kvs::StampTimerList timer_list;
        timer_list.push( m_entr_timer );

        const auto basedir = BaseClass::outputDirectory().baseDirectoryName() + "/";
        ret = timer_list.write( basedir + "ent_proc_time.csv" );
    }

    return BaseClass::dump() && ret;
}

inline void EntropyControlledAdaptor::exec( const BaseClass::SimTime sim_time )
{
    Controller::setCacheEnabled( BaseClass::isAnalysisStep() );
    Controller::push( BaseClass::objects() );

    BaseClass::incrementTimeStep();
    BaseClass::clearObjects();
}

inline void EntropyControlledAdaptor::execRendering()
{
    BaseClass::setRendTime( 0.0f );
    BaseClass::setCompTime( 0.0f );
    float save_time = 0.0f;
    float entr_time = 0.0f;

    float max_entropy = -1.0f;
    int max_index = 0;

    std::vector<float> entropies;
    std::vector<BaseClass::FrameBuffer> frame_buffers;

    if ( this->isEntropyStep() )
    {
        // Entropy evaluation
        for ( const auto& location : BaseClass::viewpoint().locations() )
        {
            // Draw and readback framebuffer
            auto frame_buffer = BaseClass::readback( location );

            // Output framebuffer to image file at the root node
            kvs::Timer timer( kvs::Timer::Start );
            if ( BaseClass::world().isRoot() )
            {
                const auto entropy = Controller::entropy( frame_buffer );
                entropies.push_back( entropy );
                frame_buffers.push_back( frame_buffer );

                if ( entropy > max_entropy )
                {
                    max_entropy = entropy;
                    max_index = location.index;
                }
            }
            timer.stop();
            entr_time += BaseClass::saveTimer().time( timer );
        }

        // Distribute the index indicates the max entropy image
        BaseClass::world().broadcast( max_index );
        Controller::setMaxIndex( max_index );

        // Output the rendering images and the heatmap of entropies.
        kvs::Timer timer( kvs::Timer::Start );
        if ( BaseClass::world().isRoot() )
        {
            if ( BaseClass::isOutputImageEnabled() )
            {
                const auto index = Controller::maxIndex();
                const auto& location = BaseClass::viewpoint().at( index );
                const auto& frame_buffer = frame_buffers[ index ];
                this->output_color_image( location, frame_buffer );
                //this->output_depth_image( location, frame_buffer );

                const auto dims = BaseClass::dims();
                this->output_heatmap( dims[2], dims[1], entropies );
                /*if ( Controller::dataQueue().empty() && Controller::previousData().empty() )
                {
                    this->output_heatmap_white( dims[2], dims[1] );
                }*/
            }
        }
        timer.stop();
        save_time += BaseClass::saveTimer().time( timer );
    }
    else
    {
        auto index = Controller::pathIndex();
        auto location = BaseClass::viewpoint().at( index );
        const auto xyz = location.position;
        auto frame_buffer = BaseClass::readback( location );

        kvs::Timer timer( kvs::Timer::Start );
        if ( BaseClass::world().rank() == BaseClass::world().root() )
        {
            if ( BaseClass::isOutputImageEnabled() )
            {
                location.index = 999999;
                if( ( xyz[0] != 0.0f ) && ( xyz[2] != 0.0f ) )
                {
                    this->output_color_image( location, frame_buffer );
                    //this->output_depth_image( location, frame_buffer );
                }
                location.index = 999000;
            }
        }
        timer.stop();
        save_time += BaseClass::saveTimer().time( timer );

        if( index == Controller::poleNum() )
        {
            const auto r = sqrt( xyz.dot( xyz ) );
            location.position = Controller::polePosition();
            const auto dims = BaseClass::dims();
            const auto n = dims[2];
            //const auto n = 30;

            const auto axis = kvs::Vec3( { 0.0f, 1.0f, 0.0f } );
            const auto angle = kvs::Math::pi;
            const auto da = angle / n;
            auto u_pole = Controller::poleUpVector();
            const auto u0 = kvs::Quaternion::Rotate( u_pole, axis, -0.5f * kvs::Math::pi );
            location.up_vector = u0;

            for( size_t i = 0; i <= n; i++ )
            {
                const auto u = kvs::Quaternion::Rotate( u0, axis, da * i );
                location.up_vector = u;
                frame_buffer = BaseClass::readback( location );

                kvs::Timer timer( kvs::Timer::Start );
                if ( BaseClass::world().rank() == BaseClass::world().root() )
                {
                    if ( BaseClass::isOutputImageEnabled() )
                    {
                        location.index += 1;
                        this->output_color_image( location, frame_buffer );
                        //this->output_depth_image( location, frame_buffer );
                    }
                }
                timer.stop();
                save_time += BaseClass::saveTimer().time( timer );
            }
        }
    }
    m_entr_timer.stamp( entr_time );
    BaseClass::saveTimer().stamp( save_time );
    BaseClass::rendTimer().stamp( BaseClass::rendTime() );
    BaseClass::compTimer().stamp( BaseClass::compTime() );
}

inline kvs::Vec3 EntropyControlledAdaptor::process( const Data& data )
{
    BaseClass::execPipeline( data );
    this->execRendering();

    const auto u = BaseClass::viewpoint().at( Controller::maxIndex() ).up_vector;
    Controller::setCrrUpVector( u );

    return BaseClass::viewpoint().at( Controller::maxIndex() ).position;
}

inline void EntropyControlledAdaptor::process( const Data& data, const InSituVis::Viewpoint& path, const size_t i )
{
    const auto current_step = BaseClass::timeStep();
    {
        // Reset time step, which is used for output filename,
        // for visualizing the stacked dataset.
        const auto L_crr = Controller::dataQueue().size();
        if ( L_crr > 0 )
        {
            const auto l = BaseClass::analysisInterval();
            const auto step = current_step - L_crr * l;
            BaseClass::setTimeStep( step );
        }

        // Stack current time step.
        const auto step = static_cast<float>( BaseClass::timeStep() );
        BaseClass::tstepList().stamp( step );

        // Execute vis. pipeline and rendering.
        Controller::setPathIndex( i );
        const auto vp = BaseClass::viewpoint();
        BaseClass::setViewpoint( path );
        BaseClass::execPipeline( data );
        this->execRendering();
        BaseClass::setViewpoint( vp );
    }
    BaseClass::setTimeStep( current_step );
}

inline void EntropyControlledAdaptor::output_color_image(
    const InSituVis::Viewpoint::Location& location,
    const BaseClass::FrameBuffer& frame_buffer )
{
    const auto size = BaseClass::outputImageSize( location );
    const auto buffer = frame_buffer.color_buffer;
    kvs::ColorImage image( size.x(), size.y(), buffer );
    image.write( BaseClass::outputFinalImageName( location ) );
}

inline void EntropyControlledAdaptor::output_depth_image(
    const InSituVis::Viewpoint::Location& location,
    const BaseClass::FrameBuffer& frame_buffer )
{
    const auto size = BaseClass::outputImageSize( location );
    const auto buffer = frame_buffer.depth_buffer;
    kvs::GrayImage image( size.x(), size.y(), buffer );
    image.write( BaseClass::outputFinalImageName( location ) );
}

inline void EntropyControlledAdaptor::output_heatmap(
    const size_t num_x,
    const size_t num_y,
    const std::vector<float>& entropies )
{
    float max = -1.0f;
    float min = -1.0f;
    for( size_t i = 0; i < entropies.size(); i++ )
    {
        if( max < 0 || max < entropies[i] ) { max = entropies[i]; }
        if( min < 0 || min > entropies[i] ) { min = entropies[i]; }
    }

    const size_t l = 10;
    const size_t width = num_x * ( l + 1 ) + 1;
    const size_t height = num_y * ( l + 1 ) + 1;
    kvs::ColorImage heatmap( width, height );
    auto cmap = kvs::ColorMap::CoolWarm();
    cmap.setRange( 0.0f, 1.0f );

    for( size_t i = 0; i <= num_x; i++ )
    {
        const size_t x = ( l + 1 ) * i;
        for( size_t j = 0; j < height; j++ )
        {
            heatmap.setPixel( x, j, kvs::RGBColor::Black() );
        }
    }
    for( size_t j = 0; j <= num_y; j++ )
    {
        const size_t y = ( l + 1 ) * j;
        for( size_t i = 0; i < width; i++ )
        {
            heatmap.setPixel( i, y, kvs::RGBColor::Black() );
        }
    }

    for( size_t j = 0; j < num_y; j++ )
    {
        for( size_t i = 0; i < num_x; i++ )
        {
            const size_t x = ( l + 1 ) * i + 1;
            const size_t y = ( l + 1 ) * j + 1;
            const size_t index = i + j * num_x;
            const float value_norm = ( entropies[ index ] - min ) / ( max - min );
            const auto c = cmap.at( value_norm );

            for( size_t n = 0; n < l; n++ )
            {
                for( size_t m = 0; m < l; m++ )
                {
                    heatmap.setPixel( x + m, y + n, c );
                }
            }
        }
    }

    const auto time = BaseClass::timeStep();
    const auto output_time = kvs::String::From( time, 6, '0' );
    const auto output_filename = "heatmap_" + output_time;
    const auto filename = BaseClass::outputDirectory().baseDirectoryName() + "/" + output_filename + ".bmp";
    
    heatmap.write( filename );
}

inline void EntropyControlledAdaptor::output_heatmap_white(
    const size_t num_x,
    const size_t num_y )
{
    const size_t l = 10;
    const size_t width = num_x * ( l + 1 ) + 1;
    const size_t height = num_y * ( l + 1 ) + 1;
    kvs::ColorImage heatmap_white( width, height );

    for( size_t i = 0; i < width; i++ )
    {
        for( size_t j = 0; j < height; j++ )
        {
            heatmap_white.setPixel( i, j, kvs::RGBColor::White() );
        }
    }

    for( size_t i = 0; i <= num_x; i++ )
    {
        const size_t x = ( l + 1 ) * i;
        for( size_t j = 0; j < height; j++ )
        {
            heatmap_white.setPixel( x, j, kvs::RGBColor::Black() );
        }
    }
    for( size_t j = 0; j <= num_y; j++ )
    {
        const size_t y = ( l + 1 ) * j;
        for( size_t i = 0; i < width; i++ )
        {
            heatmap_white.setPixel( i, y, kvs::RGBColor::Black() );
        }
    }

    const auto filename = BaseClass::outputDirectory().baseDirectoryName() + "/heatmap_white.bmp";
    
    heatmap_white.write( filename );
}

} // end of namespace mpi

} // end of namespace InSituVis
