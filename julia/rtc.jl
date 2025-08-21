using PythonCall: PythonCall, Py, pyimport, pyconvert

PYPATH = abspath(joinpath(@__DIR__, "..", "python"))

sys = pyimport("sys")
sys.path.append(PYPATH)

pyrtc_interface = pyimport("rtc_interface")

function convert_pydata(data::Py)
    converted = Dict{String, Any}()
    for _key in data.keys()
        key = pyconvert(String, _key)
        value = pyconvert(Array, data[_key])
        converted[key] = value
    end
    return converted
end

rtc_interface() = pyrtc_interface.rtc_interface()

function set_stream(rtc, stream::Int, channels::Vector{String}, duration::Int, downsample::Int = 0)
    rtc.set_stream(stream, channels, duration, downsample)
    return rtc
end

get_stream(rtc, stream::Int) = convert_pydata(rtc.get_stream(stream))
run_stream(rtc, stream::Int) = convert_pydata(rtc.run_stream(stream))
