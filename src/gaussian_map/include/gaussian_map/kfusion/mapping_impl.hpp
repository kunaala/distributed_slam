/*
 *
 * Copyright 2016 Emanuele Vespa, Imperial College London 
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 *
 * */
#ifndef KFUSION_MAPPING_HPP
#define KFUSION_MAPPING_HPP
#include <gaussian_map/node.hpp>

struct sdf_update {

	template <typename DataHandlerT>
	void operator()(DataHandlerT& handler, const float sdf) {

		// const Eigen::Vector2i px = pixel.cast<int>();
		// const float depthSample = depth[px(0) + depthSize(0)*px(1)];
		// if (depthSample <=  0) return;
		// const float diff = (depthSample - pos(2)) 
		// 	* std::sqrt( 1 + se::math::sq(pos(0) / pos(2)) + se::math::sq(pos(1) / pos(2)));
		// if (diff > -mu) {
		const float tsdf = fminf(1.f, sdf / mu);
		auto data = handler.get();
		data.x = se::math::clamp(
				(static_cast<float>(data.y) * data.x + tsdf) / (static_cast<float>(data.y) + 1.f), 
				-1.f,
				1.f);
		data.y = fminf(data.y + 1, maxweight);
		handler.set(data);
		// }
	} 

	sdf_update(float m, int mw) : 
		mu(m), maxweight(mw){};
	float mu;
	int maxweight;
};

#endif
