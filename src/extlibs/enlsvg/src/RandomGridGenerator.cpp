//#include <algorithm>
//#include "Grid.h"
//#include "RandomGridGenerator.h"
//#include "RandomNumberGenerator.h"
//#include <vector>
//
//namespace Pathfinding {
//
//void RandomGridGenerator::generateRandomGrid(Grid& grid, const float percentBlocked) {
//    const int sizeX = grid.sizeX;
//    const int sizeY = grid.sizeY;
//
//    for (int y=0; y<sizeY; ++y) {
//        for (int x=0; x<sizeX; ++x) {
//            grid.setBlocked(x, y, global_rng.bernoulli(percentBlocked));
//        }
//    }
//}
//
//void RandomGridGenerator::generateAutomataGrid(Grid& grid, const float percentBlocked, const int iterations, const float resolutionMultiplier) {
//    const int sizeX = grid.sizeX;
//    const int sizeY = grid.sizeY;
//    const int totalSize = sizeX*sizeY;
//    const int resolution = std::max((int)((sizeX+sizeY)*resolutionMultiplier/150), 1);
//
//    generateRandomGrid(grid, percentBlocked);
//
//    //std::vector<std::vector<bool>>& blocked = grid.blocked;
//    // Count: used for DP computation of number of blocked neighbours.
//    //  Note: count includes the current tile as well. We subtract it when we compare with cutoff.
//    std::vector<int> count;
//    count.resize(sizeX*sizeY, 0);
//
//    const int maxCount = (resolution*2+1) * (resolution*2+1) - 1;
//    
//    for (int itr=0;itr<iterations;++itr) {
//        runAutomataIterationBlockedBorders(resolution, grid, count);
//
//        // Adjust counts to exclude the center.
//        for (int y=0;y<sizeY;++y) {
//            for (int x=0;x<sizeX;++x) {
//                count[y*sizeX + x] = (count[y*sizeX + x] - (grid.isBlockedRaw(x, y) ? 1 : 0));
//            }
//        }
//        
//        // Compute mean.
//        long long totalCount = 0;
//
//        for (int y=0;y<sizeY;++y) {
//            for (int x=0;x<sizeX;++x) {
//                totalCount += count[y*sizeX + x];
//            }
//        }
//
//        const float mean = (float)totalCount / totalSize;
//        
//        // Compute approximately the value at the pth percentile. (where p = percentBlocked)
//        const int nBins = (int)sqrt(totalSize);
//        
//        // bin 0: <= low
//        // bin n+1: >= low+range
//        // bin i: low + d*(i-1) < x < low + d*(i)
//        std::vector<int> bins(nBins+2, 0);
//        std::vector<float> binAverage(nBins+2, 0);
//        
//        const float low = mean*0.5f;
//        const float range = (mean + maxCount)/2 - low;
//        const float d = range/nBins;
//        for (int y=0;y<sizeY;++y) {
//            for (int x=0;x<sizeX;++x) {
//                int bin = (int)((count[y*sizeX + x] - low)/d) + 1;
//                bin = std::max(std::min(bin, nBins+1), 0); // Clamp bin to [0,n+1]
//                binAverage[bin] = (binAverage[bin]*bins[bin] + count[y*sizeX + x]) / (bins[bin]+1);
//                bins[bin]++;
//            }
//        }
//
//        // cutoff = value at pth percentile (similar to median)
//        // using the value at the pth percentile allows us to maintain the same percentageBlocked each iteration.
//        int remainingCumSum = (int)(totalSize * (1-percentBlocked));
//        float cutoff = -1;
//        for (int i=0;i<nBins+2;++i) {
//            remainingCumSum -= bins[i];
//            if (remainingCumSum < 0) {
//                cutoff = binAverage[i];
//                break;
//            }
//        }
//        
//        for (int y=0;y<sizeY;++y) {
//            for (int x=0;x<sizeX;++x) {
//                grid.setBlocked(x, y, count[y*sizeX + x] >= cutoff);
//            }
//        }
//    }
//}
//
//void RandomGridGenerator::runAutomataIterationBlockedBorders(const int resolution, Grid& grid, std::vector<int>& count) {
//    /*
//     * Note: for brevity, the following code:
//     * nBlocked += (px < 0 || py < 0 || px >= sizeX || py >= sizeY || grid[py*sizeY+px]) ? 1 : 0;
//     * 
//     * Is a shortened version of:
//     * if (px < 0 || py < 0 || px >= sizeX || py >= sizeY) {
//     *     nBlocked++;
//     * } else {
//     *     nBlocked += grid[py*sizeY+px] ? 1 : 0;
//     * }
//     */
//    const int sizeX = grid.sizeX;
//    const int sizeY = grid.sizeY;
//
//    { // Base case: y = 0
//        int y = 0;
//        { // Base case: x = 0
//            int x = 0;
//            int nBlocked = 0;
//            for (int i=-resolution;i<=resolution;++i) {
//                for (int j=-resolution;j<=resolution;++j) {
//                    int px = x + i;
//                    int py = y + j;
//                    nBlocked += (px < 0 || py < 0 || px >= sizeX || py >= sizeY || grid.isBlockedRaw(px, py)) ? 1 : 0;
//                }
//            }
//            
//            count[y*sizeX + x] = nBlocked;
//        }
//
//        // y = 0, x > 0
//        for (int x=1;x<sizeX;++x) {
//            int nBlocked = count[y*sizeX + (x-1)];
//
//            { // subtract for (x-1-r,?)
//                int px = x - resolution - 1;
//                for (int j=-resolution;j<=resolution;++j) {
//                    int py = y + j;
//                    nBlocked -= (px < 0 || py < 0 || px >= sizeX || py >= sizeY || grid.isBlockedRaw(px, py)) ? 1 : 0;
//                }
//            }
//            
//            { // add for (x+r,?)
//                int px = x + resolution;
//                for (int j=-resolution;j<=resolution;++j) {
//                    int py = y + j;
//                    nBlocked += (px < 0 || py < 0 || px >= sizeX || py >= sizeY || grid.isBlockedRaw(px, py)) ? 1 : 0;
//                }
//            }
//            
//            count[y*sizeX + x] = nBlocked;
//        }
//    }
//
//    // y > 0
//    for (int y=1;y<sizeY;++y) {
//        // y > 0, x = 0
//        {
//            int x = 0;
//            int nBlocked = count[(y-1)*sizeX + x];
//
//            { // subtract for (?,y-1-r)
//                int py = y - resolution - 1;
//                for (int i=-resolution;i<=resolution;++i) {
//                    int px = x + i;
//                    nBlocked -= (px < 0 || py < 0 || px >= sizeX || py >= sizeY || grid.isBlockedRaw(px, py)) ? 1 : 0;
//                }
//            }
//            
//            { // add for (?,y+r)
//                int py = y + resolution;
//                for (int i=-resolution;i<=resolution;++i) {
//                    int px = x + i;
//                    nBlocked += (px < 0 || py < 0 || px >= sizeX || py >= sizeY || grid.isBlockedRaw(px, py)) ? 1 : 0;
//                }
//            }
//
//            count[y*sizeX + x] = nBlocked;
//        }
//        
//        // y > 0, x > 0
//        for (int x=1;x<sizeX;++x) {
//            int nBlocked = count[(y-1)*sizeX + x] + count[y*sizeX + (x-1)] - count[(y-1)*sizeX+ (x-1)];
//
//            { // add (x-1-r,y-1-r)
//                int px = x - resolution - 1;
//                int py = y - resolution - 1;
//                nBlocked += (px < 0 || py < 0 || px >= sizeX || py >= sizeY || grid.isBlockedRaw(px, py)) ? 1 : 0;
//            }
//            { // add (x+r,y+r)
//                int px = x + resolution;
//                int py = y + resolution;
//                nBlocked += (px < 0 || py < 0 || px >= sizeX || py >= sizeY || grid.isBlockedRaw(px, py)) ? 1 : 0;
//            }
//            { // subtract (x-1-r,y+r)
//                int px = x - resolution - 1;
//                int py = y + resolution;
//                nBlocked -= (px < 0 || py < 0 || px >= sizeX || py >= sizeY || grid.isBlockedRaw(px, py)) ? 1 : 0;
//            }
//            { // subtract (x+r,y-1-r)
//                int px = x + resolution;
//                int py = y - resolution - 1;
//                nBlocked -= (px < 0 || py < 0 || px >= sizeX || py >= sizeY || grid.isBlockedRaw(px, py)) ? 1 : 0;
//            }
//            
//            count[y*sizeX + x] = nBlocked;
//        }
//    }
//}
//
//}