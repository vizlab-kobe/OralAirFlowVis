
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

        // Output the rendering images.
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
            }
        }
        timer.stop();
        save_time += BaseClass::saveTimer().time( timer );
    }
    else
    {
        /*
        const auto path_index = Controller::pathIndex();
        const auto num_point = Controller::numPoint();
        size_t start_index = 0;
        if( path_index > 0 ) { for( size_t i = 0; i < path_index; i++ ) { start_index += num_point[i]; } }

        for ( size_t i = 0; i < num_point[ path_index ]; i++ )
        {
            auto location = BaseClass::viewpoint().at( start_index + i );
            if( ( location.position[0] == 0.0f ) && ( location.position[2] == 0.0f ) )
            {
                const size_t np = 60;
                const auto dp = kvs::Math::pi / static_cast<float>( np - 1 );
                const auto axis = kvs::Vec3( { 0.0f, 1.0f, 0.0f } );
                const auto u0 = location.up_vector;
                location.up_vector = kvs::Quaternion::Rotate( u0, axis, kvs::Math::pi / 2 );
                
                for( size_t j = 0; j < np; j++ )
                {
                    const auto u = location.up_vector;
                    const auto uu = kvs::Quaternion::Rotate( u, axis, -1.0f * dp * j );
                    location.up_vector = uu;
                    
                    auto frame_buffer = BaseClass::readback( location );
                    
                    kvs::Timer timer( kvs::Timer::Start );
                    if ( BaseClass::world().rank() == BaseClass::world().root() )
                    {
                        if ( BaseClass::isOutputImageEnabled() )
                        {
                            location.index = ( path_index + 1 ) * 1000 + j;
                            this->output_color_image( location, frame_buffer );
                            //this->output_depth_image( location, frame_buffer );
                        }
                    }
                    
                    timer.stop();
                    save_time += BaseClass::saveTimer().time( timer );
                }
            }
            else
            {
                auto frame_buffer = BaseClass::readback( location );
                
                kvs::Timer timer( kvs::Timer::Start );
                if ( BaseClass::world().rank() == BaseClass::world().root() )
                {
                    if ( BaseClass::isOutputImageEnabled() )
                    {
                        location.index = ( path_index + 1 ) * 1000 + i;
                        this->output_color_image( location, frame_buffer );
                        //this->output_depth_image( location, frame_buffer );
                    }
                }
                
                timer.stop();
                save_time += BaseClass::saveTimer().time( timer );
            }
        }*/

        auto index = Controller::pathIndex();
        auto location = BaseClass::viewpoint().at( index );
        auto frame_buffer = BaseClass::readback( location );

        kvs::Timer timer( kvs::Timer::Start );
        if ( BaseClass::world().rank() == BaseClass::world().root() )
        {
            if ( BaseClass::isOutputImageEnabled() )
            {
                location.index = 999999;
                this->output_color_image( location, frame_buffer );
                //this->output_depth_image( location, frame_buffer );
            }
        }

        timer.stop();
        save_time += BaseClass::saveTimer().time( timer );
    }
    m_entr_timer.stamp( entr_time );
    BaseClass::saveTimer().stamp( save_time );
    BaseClass::rendTimer().stamp( BaseClass::rendTime() );
    BaseClass::compTimer().stamp( BaseClass::compTime() );
}

/*inline void execRendering()
{
    m_rend_time = 0.0f;
    m_comp_time = 0.0f;
    float save_time = 0.0f;

    {
        const auto& location = BaseClass::viewpoint().at( index() );
        auto frame_buffer = this->readback( location );

        kvs::Timer timer( kvs::Timer::Start );

        if ( m_world.rank() == m_world.root() )
        {
            if ( BaseClass::isOutputImageEnabled() )
            {
                const auto size = BaseClass::outputImageSize( location );
                const auto index = location.index;
                const auto width = size.x();
                const auto height = size.y();
                const auto color_buffer = frame_buffer.color_buffer;
                kvs::ColorImage image( width, height, color_buffer );

                const auto time = BaseClass::timeStep();
                const auto output_time = kvs::String::From( time, 6, '0' );
                const auto output_basename = BaseClass::outputFilename();
                const auto output_filename = output_basename + "_" + output_time;
                const auto filename = BaseClass::outputDirectory().baseDirectoryName() + "/" + output_filename + ".bmp";
                image.write( filename );
            }
        }

        timer.stop();
        save_time += BaseClass::saveTimer().time( timer );
    }
    BaseClass::saveTimer().stamp( save_time );
    BaseClass::rendTimer().stamp( m_rend_time );
    m_comp_timer.stamp( m_comp_time );
}*/

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

} // end of namespace mpi

} // end of namespace InSituVis
