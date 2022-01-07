
namespace local
{

namespace mpi
{

inline bool EntropyControlledAdaptor::isEntropyStep()
{
    return BaseClass::timeStep() % ( BaseClass::analysisInterval() * Controller::entropyInterval() ) == 0;
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
    
    size_t max_index = 0;
    float max_value = -1;

    kvs::ValueArray<kvs::ColorImage> images( BaseClass::viewpoint().numberOfLocations() );
    kvs::ValueArray<float> entropy( BaseClass::viewpoint().numberOfLocations() );

    if( this->isEntropyStep() )
    {
        for ( const auto& location : BaseClass::viewpoint().locations() )
        {
            // Draw and readback framebuffer
            auto frame_buffer = BaseClass::readback( location );

            // Output framebuffer to image file at the root node
            kvs::Timer timer( kvs::Timer::Start );
            if ( BaseClass::world().rank() == BaseClass::world().root() )
            {
                if ( BaseClass::isOutputImageEnabled() )
                {
                    const auto size = BaseClass::outputImageSize( location );
                    const auto index = location.index;
                    const auto width = size.x();
                    const auto height = size.y();
                    const auto color_buffer = frame_buffer.color_buffer;
                    const auto depth_buffer = frame_buffer.depth_buffer;
                    kvs::ColorImage image( width, height, color_buffer );
                    entropy[ index ] = Controller::DepthEntropy( width, height, color_buffer, depth_buffer );
                    images[ index ] = image;
                    //image.write( this->outputFinalImageName( location ) );

                    if( ( max_value < 0 ) || ( max_value < entropy[ index ] ) ){
                        max_value = entropy[ index ];
                        max_index = index;
                    }
                }
            }
            timer.stop();
            save_time += BaseClass::saveTimer().time( timer );
        }

        Controller::setMaxIndex( max_index ); 
        if ( BaseClass::world().rank() == BaseClass::world().root() )
        {
            const auto time = BaseClass::timeStep();
            const auto space = max_index;
            const auto output_time = kvs::String::From( time, 6, '0' );
            const auto output_space = kvs::String::From( space, 6, '0' );
            const auto output_basename = BaseClass::outputFilename();
            const auto output_filename = output_basename + "_" + output_time + "_" + output_space;
            const auto filename = BaseClass::outputDirectory().baseDirectoryName() + "/" + output_filename + ".bmp";
            images[ space ].write( filename );
        }
    }
    else
    {
        const auto& location = BaseClass::viewpoint().at( pathIndex() );
        auto frame_buffer = BaseClass::readback( location );
            
        kvs::Timer timer( kvs::Timer::Start );

        if ( BaseClass::world().rank() == BaseClass::world().root() )
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

inline void EntropyControlledAdaptor::process( const Data& data , const InSituVis::Viewpoint& path, const size_t i )
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

} // end of namespace mpi

} // end of namespace InSituVis
