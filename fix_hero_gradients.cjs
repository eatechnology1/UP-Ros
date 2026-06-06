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
            // Replace dark gradients
            .replace(/linear-gradient\([^,]+,\s*#0f172a[^,]*,\s*#1e293b[^)]*\)/gi, 'linear-gradient(135deg, var(--bg-surface) 0%, var(--bg-surface-solid) 100%)');
            
        return match.replace(styleContent, newStyle);
    });

    if (content !== original) {
        fs.writeFileSync(file, content);
        changedFiles++;
        console.log("Updated", file);
    }
});

console.log(`Changed ${changedFiles} files.`);
