const path = require("path")

module.exports = {
    mode: "development", // "production" | "development" | "none"
    // Chosen mode tells webpack to use its built-in optimizations accordingly.
    // defaults to ./src
    optimization: {
        // chunkIds: "size",
        // method of generating ids for chunks
        // moduleIds: "size",
        // method of generating ids for modules
        // mangleExports: "size",
        // rename export names to shorter names
        minimize: false,
        removeAvailableModules: false,
        removeEmptyChunks: false,
        splitChunks: false,
        // minimize the output files
        /* Advanced optimizations (click to show) */
    },
    output: {
        path: path.resolve(__dirname, '../')
    }
    /* Advanced configuration (click to show) */
    /* Advanced caching configuration (click to show) */
    /* Advanced build configuration (click to show) */
}