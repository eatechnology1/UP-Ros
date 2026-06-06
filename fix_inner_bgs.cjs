const fs = require('fs');
const path = require('path');

function walk(dir) {
    let results = [];
    const list = fs.readdirSync(dir);
    list.forEach(function(file) {
        file = path.join(dir, file);
        const stat = fs.statSync(file);
        if (stat && stat.isDirectory()) { 
            results = results.concat(walk(file));
        } else if (file.endsWith('.vue')) {
            results.push(file);
        }
    });
    return results;
}

const vueFiles = walk('./src/pages').concat(walk('./src/components'));
let changedFiles = 0;

vueFiles.forEach(file => {
    let content = fs.readFileSync(file, 'utf8');
    let original = content;

    const styleRegex = /<style[^>]*>([\s\S]*?)<\/style>/g;

    content = content.replace(styleRegex, (match, styleContent) => {
        let newStyle = styleContent
            // Replace background: rgba(0, 0, 0, 0.x)
            .replace(/background:\s*rgba\(\s*0\s*,\s*0\s*,\s*0\s*,\s*0\.[1-9]\s*\)/g, 'background: var(--bg-surface-hover)')
            // Replace background: rgba(255, 255, 255, 0.x)
            .replace(/background:\s*rgba\(\s*255\s*,\s*255\s*,\s*255\s*,\s*0\.[0-9]+\s*\)/g, 'background: var(--bg-surface-hover)');
            
        return match.replace(styleContent, newStyle);
    });

    if (content !== original) {
        fs.writeFileSync(file, content);
        changedFiles++;
        console.log("Updated", file);
    }
});

console.log(`Changed ${changedFiles} files.`);
